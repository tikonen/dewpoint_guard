#pragma once

template <class T>
struct LowPassFilter {
    T filtered;
    float ratio;

    LowPassFilter(T initial, float r)
        : filtered(initial)
        , ratio(r)
    {
    }

    void reset(T val) { filtered = val; }

    T update(T val)
    {
        const T r = round((1 - ratio) * filtered + val * ratio);
        filtered = r;
        return r;
    }
};
