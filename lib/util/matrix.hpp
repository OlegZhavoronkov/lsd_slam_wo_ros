#pragma once

// Work in progress...

// Matrix uses X,Y ordering
template <class T, size_t WIDTH, size_t HEIGHT>
using Matrix = std::array<std::array<T, HEIGHT>, WIDTH>;
