#pragma once

#include <cstdlib>
#include <cstdint>
#include <LinkedList.h>

#include "ws2812.h"

template <size_t stripCount, size_t stripLength, size_t stripOffset = 0>
struct LEDMatrix {
    static const int STRIP_COUNT = stripCount;
    static const int STRIP_LENGTH = stripLength;
    static const int STRIP_OFFSET = stripOffset;

    static inline size_t getIndex(size_t x, size_t y) {
        // TODO assert
        if (x >= stripCount || y > stripLength) {
            return 0;
        }
        size_t startIndex = x * (stripLength + stripOffset);
        return startIndex + (x % 2 ? y : stripLength - y - 1);
    }
};

class LEDRange {
public:
    virtual void setPixel(size_t index, uint8_t r, uint8_t g, uint8_t b, bool upsideDown = false) {
        WS2812_framedata_setPixel(6, getIndex(index, upsideDown), r, g, b);
    }

    virtual size_t getSize() const = 0;
    virtual size_t getIndex(size_t index, bool upsideDown = false) const = 0;
};

template <typename Mapping>
class LEDRow : public LEDRange {
public:
    LEDRow(size_t row = 0)
        : row(row) {}

    virtual size_t getSize() const {
        return SIZE;
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        return Mapping::getIndex(upsideDown ? getSize() - index - 1 : index, row);
    }

    static const size_t SIZE = Mapping::STRIP_COUNT;
    static const size_t COUNT = Mapping::STRIP_LENGTH;

protected:
    size_t row;
};

template <typename Mapping>
class LEDColumn : public LEDRange {
public:
    LEDColumn(size_t column = 0)
        : column(column) {}

    virtual size_t getSize() const {
        return SIZE;
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        return Mapping::getIndex(column, upsideDown ? getSize() - index - 1 : index);
    }

    static const size_t SIZE = Mapping::STRIP_LENGTH;
    static const size_t COUNT = Mapping::STRIP_COUNT;

protected:
    size_t column;
};

class MultipleLEDRanges : public LEDRange {
public:
    MultipleLEDRanges() {}

    void addRange(LEDRange* range) {
        ranges.add(range);
    }

    virtual size_t getSize() const {
        ASSERT(ranges->hasValue());
        return ranges.value->getSize();
    }

    virtual size_t getIndex(size_t index, bool upsideDown = false) const {
        return 0;
    }

    virtual void setPixel(size_t index, uint8_t r, uint8_t g, uint8_t b, bool upsideDown) {
        for (LinkedListIterator<LEDRange*> it(ranges); !it.end(); it.next()) {
            (*it)->setPixel(index, r, g, b, upsideDown);
        }
    }

protected:
    LinkedList<LEDRange*> ranges;
};

