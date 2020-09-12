#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/tnm067lab1/processors/imageupsampler.h>
#include <modules/tnm067lab1/utils/interpolationmethods.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/imageramutils.h>

#include <array>

namespace inviwo {

namespace detail {

template <typename T>
void upsample(ImageUpsampler::IntepolationMethod method, const LayerRAMPrecision<T>& inputImage,
              LayerRAMPrecision<T>& outputImage) {
    using F = typename float_type<T>::type;

    const size2_t inputSize = inputImage.getDimensions();
    const size2_t outputSize = outputImage.getDimensions();

    const T* inPixels = inputImage.getDataTyped();
    T* outPixels = outputImage.getDataTyped();

    auto inIndex = [&inputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(inputSize - size2_t(1)));
        return pos.x + pos.y * inputSize.x;
    };
    auto outIndex = [&outputSize](auto pos) -> size_t {
        pos = glm::clamp(pos, decltype(pos)(0), decltype(pos)(outputSize - size2_t(1)));
        return pos.x + pos.y * outputSize.x;
    };

    util::forEachPixel(outputImage, [&](ivec2 outImageCoords) {
        // outImageCoords: Exact pixel coordinates in the output image currently writing to
        // inImageCoords: Relative coordinates of outImageCoords in the input image, might be
        // between pixels
        dvec2 inImageCoords =
            ImageUpsampler::convertCoordinate(outImageCoords, inputSize, outputSize);

        T finalColor(0);

        // DUMMY COLOR, remove or overwrite this bellow
        finalColor = inPixels[inIndex(
            glm::clamp(size2_t(outImageCoords), size2_t(0), size2_t(inputSize - size2_t(1))))];

        switch (method) {
            case ImageUpsampler::IntepolationMethod::PiecewiseConstant: {
                // Task 8
                // Update finalColor
                finalColor = inPixels[inIndex(floor(inImageCoords))];
                break;
            }
            case ImageUpsampler::IntepolationMethod::Bilinear: {
                // inImageCoords
                dvec2 black_dot = inImageCoords - 0.5;
                ivec2 black_dot_floored = ivec2(black_dot);

                // Get neighbour pixels
                std::array<T, 4> values = { 
                    inPixels[inIndex(black_dot_floored)],
                    inPixels[inIndex(black_dot_floored + ivec2(1, 0))],
                    inPixels[inIndex(black_dot_floored + ivec2(0, 1))],
                    inPixels[inIndex(black_dot_floored + ivec2(1, 1))]
                };

                // Update finalColor
                finalColor = TNM067::Interpolation::bilinear(values, black_dot.x - (int)black_dot.x, black_dot.y - (int)black_dot.y);

                break;
            }
            case ImageUpsampler::IntepolationMethod::Quadratic: {
                // Update finalColor

                // inImageCoords
                dvec2 black_dot = inImageCoords - 0.5;
                ivec2 black_dot_floored = ivec2(black_dot);

                // Get neighbour pixels
                std::array<T, 9> values = { 
                    inPixels[inIndex(black_dot_floored)],
                    inPixels[inIndex(black_dot_floored + ivec2(1, 0))],
                    inPixels[inIndex(black_dot_floored + ivec2(2, 0))],
                    inPixels[inIndex(black_dot_floored + ivec2(0, 1))],
                    inPixels[inIndex(black_dot_floored + ivec2(1, 1))],
                    inPixels[inIndex(black_dot_floored + ivec2(2, 1))],
                    inPixels[inIndex(black_dot_floored + ivec2(0, 2))],
                    inPixels[inIndex(black_dot_floored + ivec2(1, 2))],
                    inPixels[inIndex(black_dot_floored + ivec2(2, 2))]
                };

                // Update finalColor
                double dx = black_dot.x - (int)black_dot.x;
                double dy = black_dot.y - (int)black_dot.y;

                finalColor = TNM067::Interpolation::biQuadratic(values, dx / 2.0, dy / 2.0);

                break;
            }
            case ImageUpsampler::IntepolationMethod::Barycentric: {
                // inImageCoords
                dvec2 p = inImageCoords - 0.5;
                ivec2 p_floored = ivec2(p);

                // Get neighbour pixels
                std::array<T, 4> values = {inPixels[inIndex(p_floored)],
                                           inPixels[inIndex(p_floored + ivec2(1, 0))],
                                           inPixels[inIndex(p_floored + ivec2(0, 1))],
                                           inPixels[inIndex(p_floored + ivec2(1, 1))]};
                
                // Update finalColor
                finalColor = inviwo::TNM067::Interpolation::barycentric(values, p.x - (int)p.x,
                                                                        p.y - (int)p.y);


                break;
            }
            default:
                break;
        }

        outPixels[outIndex(outImageCoords)] = finalColor;
    });
}

}  // namespace detail

const ProcessorInfo ImageUpsampler::processorInfo_{
    "org.inviwo.imageupsampler",  // Class identifier
    "Image Upsampler",            // Display name
    "TNM067",                     // Category
    CodeState::Experimental,      // Code state
    Tags::None,                   // Tags
};
const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

ImageUpsampler::ImageUpsampler()
    : Processor()
    , inport_("inport", true)
    , outport_("outport", true)
    , interpolationMethod_("interpolationMethod", "Interpolation Method",
                           {
                               {"piecewiseconstant", "Piecewise Constant (Nearest Neighbor)",
                                IntepolationMethod::PiecewiseConstant},
                               {"bilinear", "Bilinear", IntepolationMethod::Bilinear},
                               {"quadratic", "Quadratic", IntepolationMethod::Quadratic},
                               {"barycentric", "Barycentric", IntepolationMethod::Barycentric},
                           }) {
    addPort(inport_);
    addPort(outport_);
    addProperty(interpolationMethod_);
}

void ImageUpsampler::process() {
    auto inputImage = inport_.getData();
    if (inputImage->getDataFormat()->getComponents() != 1) {
        LogError("The ImageUpsampler processor does only support single channel images");
    }

    auto inSize = inport_.getData()->getDimensions();
    auto outDim = outport_.getDimensions();

    auto outputImage = std::make_shared<Image>(outDim, inputImage->getDataFormat());
    outputImage->getColorLayer()->setSwizzleMask(inputImage->getColorLayer()->getSwizzleMask());
    outputImage->getColorLayer()
        ->getEditableRepresentation<LayerRAM>()
        ->dispatch<void, dispatching::filter::Scalars>([&](auto outRep) {
            auto inRep = inputImage->getColorLayer()->getRepresentation<LayerRAM>();
            detail::upsample(interpolationMethod_.get(), *(const decltype(outRep))(inRep), *outRep);
        });

    outport_.setData(outputImage);
}

dvec2 ImageUpsampler::convertCoordinate(ivec2 outImageCoords, [[maybe_unused]] size2_t inputSize,
                                        [[maybe_unused]] size2_t outputsize) {
    // TODO implement
    dvec2 c(outImageCoords);

    // TASK 7: Convert the outImageCoords to its coordinates in the input image
    c = c * (dvec2(inputSize)/dvec2(outputsize));

    return c;
}

}  // namespace inviwo
