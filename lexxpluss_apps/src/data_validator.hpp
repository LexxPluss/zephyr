#pragma once

#include <cfloat>
#include <cmath>

template <typename T, uint32_t N>
class validator_base {
public:
    void put(T *data) {
        for (uint32_t i{0}; i < N; ++i) {
            if (value[i] == data[i])
                ++equal_count;
            else
                equal_count = 0;
            value[i] = data[i];
        }
    }
    T get(uint32_t index = 0) const {
        return value[index];
    }
    bool is_low_confidence() const {
        return confidence() < 0.5f;
    }
    uint32_t equal_count{0};
    static constexpr uint32_t MAX_EQUAL_COUNT{100};
protected:
    float confidence() const {
        float result{1.0f};
        if (equal_count > MAX_EQUAL_COUNT)
            result = 0.0f;
        return result;
    }
    T value[N]{0};
};

template <typename T, uint32_t N>
struct data_validator : public validator_base<T, N> {
};

template <typename T>
class data_validator<T, 1> : public validator_base<T, 1> {
public:
    void put(T val) {
        parent::put(&val);
    }
private:
    typedef validator_base<T, 1> parent;
};

template <uint32_t N>
class data_validator<float, N> : public validator_base<float, N> {
public:
    void put(float *data) {
        for (uint32_t i{0}; i < N; ++i) {
            if (fabsf(parent::value[i] - data[i]) < FLT_EPSILON)
                ++parent::equal_count;
            else
                parent::equal_count = 0;
            parent::value[i] = data[i];
        }
    }
private:
    typedef validator_base<float, N> parent;
};

template <>
class data_validator<float, 1> : public validator_base<float, 1> {
public:
    void put(float val) {
        parent::put(&val);
    }
private:
    typedef validator_base<float, 1> parent;
};

// vim: set expandtab shiftwidth=4:
