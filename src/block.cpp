#include <nori/block.h>
#include <nori/bitmap.h>
#include <nori/rfilter.h>
#include <nori/bbox.h>
#include <tbb/tbb.h>

NORI_NAMESPACE_BEGIN

ImageBlock::ImageBlock(const Vector2i &outputSize, const ReconstructionFilter *filter) 
        : m_offset(0, 0), m_outputSize(outputSize) {
    if (filter) {
        /* Tabulate the image reconstruction filter for performance reasons */
        m_filterRadius = filter->getRadius();
        m_borderSize = (int)std::ceil(m_filterRadius - 0.5f); // rounding m_filterRadius
        m_filter = new float[NORI_FILTER_RESOLUTION + 1];
        for (int i = 0; i < NORI_FILTER_RESOLUTION; ++i)
        {
            float pos = (m_filterRadius * i) / NORI_FILTER_RESOLUTION; // Sampling in turn within the radius
            m_filter[i] = filter->eval(pos);
        }
        m_filter[NORI_FILTER_RESOLUTION] = 0.0f;
        m_lookupFactor = NORI_FILTER_RESOLUTION / m_filterRadius;
        // Weights of the corresponding dimension in the filtering range includes centor pixel
        int weightSize = (int)std::ceil(2 * m_filterRadius) + 1;
        m_weightsX = new float[weightSize];
        m_weightsY = new float[weightSize];
        memset(m_weightsX, 0, sizeof(float) * weightSize);
        memset(m_weightsY, 0, sizeof(float) * weightSize);
    }

    /* Allocate space for pixels and border regions */
    resize(outputSize.y() + 2 * m_borderSize, outputSize.x() + 2 * m_borderSize); // The filtering range may exceed the original image boundary
}

ImageBlock::~ImageBlock() {
    delete[] m_filter;
    delete[] m_weightsX;
    delete[] m_weightsY;
}

Bitmap *ImageBlock::toBitmap() const { // from Block to Bitmap
    Bitmap *result = new Bitmap(m_outputSize);
    for (int y=0; y<m_outputSize.y(); ++y)
        for (int x=0; x<m_outputSize.x(); ++x)
            result->coeffRef(y, x) = coeff(y + m_borderSize, x + m_borderSize).divideByFilterWeight();
    return result;
}

void ImageBlock::fromBitmap(const Bitmap &bitmap) { // from Bitmap to Block
    if (bitmap.cols() != cols() || bitmap.rows() != rows())
        throw NoriException("Invalid bitmap dimensions!");

    for (int y=0; y<m_outputSize.y(); ++y)
        for (int x=0; x<m_outputSize.x(); ++x)
            coeffRef(y, x) << bitmap.coeff(y, x), 1;
}

void ImageBlock::put(const Point2f &_pos, const Color3f &value) {
    if (!value.isValid()) {
        /* If this happens, go fix your code instead of removing this warning ;) */
        cerr << "Integrator: computed an invalid radiance value: " << value.toString() << endl;
        return;
    }

    /* Convert to pixel coordinates within the image block */
    Point2f pos(
        _pos.x() - 0.5f - (m_offset.x() - m_borderSize),
        _pos.y() - 0.5f - (m_offset.y() - m_borderSize)
    );

    /* Compute the rectangle of pixels that will need to be updated */
    BoundingBox2i bbox(
        Point2i((int)  std::ceil(pos.x() - m_filterRadius), (int)  std::ceil(pos.y() - m_filterRadius)),
        Point2i((int) std::floor(pos.x() + m_filterRadius), (int) std::floor(pos.y() + m_filterRadius))
    );
    bbox.clip(BoundingBox2i(Point2i(0, 0), Point2i((int)cols() - 1, (int)rows() - 1)));

    /* Lookup values from the pre-rasterized filter */
    for (int x = bbox.min.x(), idx = 0; x <= bbox.max.x(); ++x)
        m_weightsX[idx++] = m_filter[(int)(std::abs(x - pos.x()) * m_lookupFactor)];
    for (int y = bbox.min.y(), idx = 0; y <= bbox.max.y(); ++y)
        m_weightsY[idx++] = m_filter[(int)(std::abs(y - pos.y()) * m_lookupFactor)];

    for (int y = bbox.min.y(), yr = 0; y <= bbox.max.y(); ++y, ++yr)
        for (int x = bbox.min.x(), xr = 0; x <= bbox.max.x(); ++x, ++xr)
            coeffRef(y, x) += Color4f(value) * m_weightsX[xr] * m_weightsY[yr];
}
    
void ImageBlock::put(ImageBlock &b) {
    Vector2i offset = b.getOffset() - m_offset;
        //  + Vector2i::Constant(m_borderSize - b.getBorderSize());
    Vector2i size   = b.getSize()   + Vector2i(2*b.getBorderSize());

    tbb::mutex::scoped_lock lock(m_mutex); // During the merge operation, this function locks the destination block using a mutex.

    block(offset.y(), offset.x(), size.y(), size.x()) 
        += b.topLeftCorner(size.y(), size.x());
}

std::string ImageBlock::toString() const {
    return tfm::format("ImageBlock[offset=%s, size=%s]]",
        m_offset.toString(), m_outputSize.toString());
}

BlockGenerator::BlockGenerator(const Vector2i &outputSize, int blockSize)
        : m_outputSize(outputSize), m_blockSize(blockSize) {
    m_numBlocks = Vector2i(
        (int) std::ceil(outputSize.x() / (float) blockSize),
        (int) std::ceil(outputSize.y() / (float) blockSize));
    m_blocksLeft = m_numBlocks.x() * m_numBlocks.y();
    m_direction = ERight;
    m_block = Point2i(m_numBlocks / 2); // start from center
    m_stepsLeft = 1;
    m_numSteps = 1;
}

bool BlockGenerator::next(ImageBlock &block) {
    tbb::mutex::scoped_lock lock(m_mutex);

    if (m_blocksLeft == 0)
        return false;

    Point2i pos = m_block * m_blockSize;
    block.setOffset(pos);
    // If the remaining part is large enough, it will be updated with the preset size, otherwise it will be cropped
    block.setSize((m_outputSize - pos).cwiseMin(Vector2i::Constant(m_blockSize)));

    if (--m_blocksLeft == 0)
        return true;

    do {
        switch (m_direction) { // changes every step
            case ERight: ++m_block.x(); break;
            case EDown:  ++m_block.y(); break;
            case ELeft:  --m_block.x(); break;
            case EUp:    --m_block.y(); break;
        }
        if (--m_stepsLeft == 0) { // left steps in this turn
            m_direction = (m_direction + 1) % 4;
            if (m_direction == ELeft || m_direction == ERight) 
                ++m_numSteps;  // changes every two times the direction switch
            m_stepsLeft = m_numSteps;
        }
    } while ((m_block.array() < 0).any() ||
             (m_block.array() >= m_numBlocks.array()).any()); // until out of this block's scope

    return true;
}

NORI_NAMESPACE_END
