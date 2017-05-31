#pragma once

#include <cstdlib>
#include <cstdint>
#include <LinkedList.h>
#include <rassert.h>

#include "ws2812.h"

template <size_t stripCount, size_t stripLength, size_t stripOffset = 0>
struct WS2812Matrix {
    static const int STRIP_COUNT = stripCount;
    static const int STRIP_LENGTH = stripLength;
    static const int STRIP_OFFSET = stripOffset;

    static inline size_t getIndex(size_t x, size_t y) {
        RASSERT(x < stripCount && y < stripLength);
        size_t startIndex = x * (stripLength + stripOffset);
        return startIndex + (x % 2 ? y : stripLength - y - 1);
    }
};

class WS2812Span {
public:
    virtual void setPixel(size_t index, uint8_t r, uint8_t g, uint8_t b, bool upsideDown = false) {
        // TODO led pin
        WS2812_framedata_setPixel(6, getIndex(index, upsideDown), r, g, b);
    }

    virtual size_t getSize() const = 0;
    virtual size_t getIndex(size_t index, bool upsideDown = false) const = 0;
};

template <typename Mapping>
class WS2812Row : public WS2812Span {
public:
    WS2812Row(size_t row = 0)
        : row(row) {
        RASSERT(row < COUNT);
    }

    virtual size_t getSize() const {
        return SIZE;
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        RASSERT(index < getSize());
        return Mapping::getIndex(upsideDown ? getSize() - index - 1 : index, row);
    }

    static const size_t SIZE = Mapping::STRIP_COUNT;
    static const size_t COUNT = Mapping::STRIP_LENGTH;

protected:
    size_t row;
};

template <typename Mapping>
class WS2812Column : public WS2812Span {
public:
    WS2812Column(size_t column = 0)
        : column(column) {
        RASSERT(column < COUNT);
    }

    virtual size_t getSize() const {
        return SIZE;
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        RASSERT(index < getSize());
        return Mapping::getIndex(column, upsideDown ? getSize() - index - 1 : index);
    }

    static const size_t SIZE = Mapping::STRIP_LENGTH;
    static const size_t COUNT = Mapping::STRIP_COUNT;

protected:
    size_t column;
};

class MultipleWS2812Spans : public WS2812Span {
public:
    MultipleWS2812Spans() {}

    void addRange(WS2812Span* range) {
        RASSERT(range != nullptr);
        ranges.add(range);
    }

    virtual size_t getSize() const {
        RASSERT(ranges.hasValue());
        return ranges.value->getSize();
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        RASSERT(false);
        return 0;
    }

    virtual void setPixel(size_t index, uint8_t r, uint8_t g, uint8_t b, bool upsideDown) {
        for (LinkedListIterator<WS2812Span*> it(ranges); !it.end(); it.next()) {
            (*it)->setPixel(index, r, g, b, upsideDown);
        }
    }

protected:
    LinkedList<WS2812Span*> ranges;
};

