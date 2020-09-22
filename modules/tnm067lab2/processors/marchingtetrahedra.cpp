#include <modules/tnm067lab2/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/network/networklock.h>

namespace inviwo {

size_t MarchingTetrahedra::HashFunc::max = 1;

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Stable,                // Code state
    Tags::None,                       // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.5f, 0.0f, 1.0f) {

    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    isoValue_.setSerializationMode(PropertySerializationMode::All);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        const auto vr = volume_.getData()->dataMap_.valueRange;
        isoValue_.setMinValue(static_cast<float>(vr.x));
        isoValue_.setMaxValue(static_cast<float>(vr.y));
        isoValue_.setIncrement(static_cast<float>(glm::abs(vr.y - vr.x) / 50.0));
        isoValue_.set(static_cast<float>(iso * (vr.y - vr.x) + vr.x));
        isoValue_.setCurrentStateAsDefault();
    });
}

/*------------------- AUXALLIARY FUNC ---------------------------*/
size_t MarchingTetrahedra::calcTriangleVert(MeshHelper& mesh, const MarchingTetrahedra::Voxel& voxel0,
                                            const MarchingTetrahedra::Voxel& voxel1, const float& iso) {
    vec3 v = voxel0.pos + ((voxel1.pos - voxel0.pos) * (iso - voxel0.value)) / (voxel1.value - voxel0.value);

    return mesh.addVertex(v, voxel0.index, voxel1.index);
}

void MarchingTetrahedra::calcTriangle(MarchingTetrahedra::MeshHelper &mesh, const float& iso, 
	const MarchingTetrahedra::Voxel& vox0, const MarchingTetrahedra::Voxel& vox1,
	const MarchingTetrahedra::Voxel& vox2, const MarchingTetrahedra::Voxel& vox3, 
	const MarchingTetrahedra::Voxel& vox4, const MarchingTetrahedra::Voxel& vox5) {

        size_t v0 = calcTriangleVert(mesh, vox0, vox1, iso);
        size_t v1 = calcTriangleVert(mesh, vox2, vox3, iso);
        size_t v2 = calcTriangleVert(mesh, vox4, vox5, iso);

        mesh.addTriangle(v0, v1, v2);
}

/*----------------------------------------------------------------*/

void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto& dims = volume->getDimensions();
    //MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

    const float iso = isoValue_.get();

    util::IndexMapper3D index(dims);

    const static size_t tetrahedraIds[6][4] = {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
                                               {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}};

    size3_t max{dims - size3_t{1}};
    size3_t pos{};
    for (pos.z = 0; pos.z < max.x; ++pos.z) {
        for (pos.y = 0; pos.y < max.y; ++pos.y) {
            for (pos.x = 0; pos.x < max.x; ++pos.x) {
                // Step 1: create current cell
                // Use volume->getAsDouble to query values from the volume
                // Spatial position should be between 0 and 1
                // The voxel index should be the 1D-index for the voxel

                Cell c;
                int voxelIndex{0};
                for (int z = 0; z < 2; ++z) {
                    for (int y = 0; y < 2; ++y) {
                        for (int x = 0; x < 2; ++x) {
                            // pos == (0, 0, 0)

                            size3_t current{x + pos.x, y + pos.y, z + pos.z};
                            //vec3 voxelPos = static_cast<vec3>(current) / static_cast<vec3>(max);
                            vec3 voxelPos{(pos.x + x) / (dims.x - 1.0), (pos.y + y) / (dims.y - 1.0), (pos.z + z) / (dims.z - 1.0)};
                 
                            c.voxels[voxelIndex++] = { voxelPos, static_cast<float>(volume->getAsDouble(current)), index(current)};
                        }
                    }
                }
                
                // Step 2: Subdivide cell into tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;

                for (size_t s{0}; s < 6; ++s) {
                    Tetrahedra tetra;
                    for (size_t t{0}; t < 4; ++t) {
                        tetra.voxels[t] = c.voxels[tetrahedraIds[s][t]];
                    }

                    tetrahedras.push_back(tetra);
                }

                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // Step three: Calculate for tetra case index
                    int caseId = 0;

                    const auto& voxel0 = tetrahedra.voxels[0];
                    const auto& voxel1 = tetrahedra.voxels[1];
                    const auto& voxel2 = tetrahedra.voxels[2];
                    const auto& voxel3 = tetrahedra.voxels[3];

                    if (voxel0.value < iso) caseId |= 1;
                    if (voxel1.value < iso) caseId |= 2;
                    if (voxel2.value < iso) caseId |= 4;
                    if (voxel3.value < iso) caseId |= 8;

                    // std::bitset<4> caseId(caseId);
                    size_t v0{}, v1{}, v2{}, v3{}, v4{}, v5{};

                    if (caseId == 1 || caseId == 14) {

                        if (caseId == 1) {
                            calcTriangle(mesh, iso, voxel0, voxel1, voxel0, voxel3, voxel0, voxel2);
                        } else {
                            calcTriangle(mesh, iso, voxel0, voxel1, voxel0, voxel2, voxel0, voxel3);
                        }
            
                    } else if (caseId == 2 || caseId == 13) {

                        if (caseId == 2) {
                            calcTriangle(mesh, iso, voxel1, voxel0, voxel1, voxel2, voxel1, voxel3);
                        } else {
                            calcTriangle(mesh, iso, voxel1, voxel0, voxel1, voxel3, voxel1, voxel2);
                        }

                    } else if (caseId == 3 || caseId == 12) {

                        if (caseId == 3) {
                            calcTriangle(mesh, iso, voxel1, voxel2, voxel1, voxel3, voxel0, voxel3);
                            calcTriangle(mesh, iso, voxel1, voxel2, voxel0, voxel3, voxel0, voxel2);
                        } else {
                            calcTriangle(mesh, iso, voxel1, voxel2, voxel0, voxel3, voxel1, voxel3);
                            calcTriangle(mesh, iso, voxel1, voxel2, voxel0, voxel2, voxel0, voxel3);
                        }
                            
                    } else if (caseId == 4 || caseId == 11) {

                        if (caseId == 4) {
                            calcTriangle(mesh, iso, voxel2, voxel3, voxel2, voxel1, voxel2, voxel0);
                        } else {
                            calcTriangle(mesh, iso, voxel2, voxel3, voxel2, voxel0, voxel2, voxel1);
                        }

                    } else if (caseId == 5 || caseId == 10) {

                        if (caseId == 5) {
                            calcTriangle(mesh, iso, voxel2, voxel1, voxel0, voxel1, voxel0, voxel3);
                            calcTriangle(mesh, iso, voxel2, voxel3, voxel2, voxel1, voxel0, voxel3);
                        } else {
                            calcTriangle(mesh, iso, voxel2, voxel1, voxel0, voxel3, voxel0, voxel1);
                            calcTriangle(mesh, iso, voxel2, voxel3, voxel0, voxel3, voxel2, voxel1);
                        }
                    } else if (caseId == 6 || caseId == 9) {

                        if (caseId == 6) {
                            calcTriangle(mesh, iso, voxel2, voxel0, voxel1, voxel3, voxel1, voxel0);
                            calcTriangle(mesh, iso, voxel2, voxel0, voxel2, voxel3, voxel1, voxel3);
                        } else {
                            calcTriangle(mesh, iso, voxel2, voxel0, voxel1, voxel0, voxel1, voxel3);
                            calcTriangle(mesh, iso, voxel2, voxel0, voxel1, voxel3, voxel2, voxel3);
                        }
                    } else if (caseId == 7 || caseId == 8) {

                        if (caseId == 7) {
                           calcTriangle(mesh, iso, voxel3, voxel1, voxel3, voxel0, voxel3, voxel2);
                        } else {
                            calcTriangle(mesh, iso, voxel3, voxel1, voxel3, voxel2, voxel3, voxel0);
                        }
                    }
                }
            }
        }
    }

    mesh_.setData(mesh.toBasicMesh());
}

MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    IVW_ASSERT(i0 != i1, "i0 and i1 should not be the same value");
    IVW_ASSERT(i0 != i2, "i0 and i2 should not be the same value");
    IVW_ASSERT(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    const auto a = std::get<0>(vertices_[i0]);
    const auto b = std::get<0>(vertices_[i1]);
    const auto c = std::get<0>(vertices_[i2]);

    const vec3 n = glm::normalize(glm::cross(b - a, c - a));
    std::get<1>(vertices_[i0]) += n;
    std::get<1>(vertices_[i1]) += n;
    std::get<1>(vertices_[i2]) += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        std::get<1>(vertex) = glm::normalize(std::get<1>(vertex));
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    IVW_ASSERT(i != j, "i and j should not be the same value");
    if (j < i) std::swap(i, j);

    auto [edgeIt, inserted] = edgeToVertex_.try_emplace(std::make_pair(i, j), vertices_.size());
    if (inserted) {
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
    }
    return static_cast<std::uint32_t>(edgeIt->second);
}

}  // namespace inviwo
