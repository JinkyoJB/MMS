/**
 * mms/sensor/artec/artec_common.h
 *
 * Shared utilities included by artec_binding.cpp and artec_base_binding.cpp.
 * All functions are static so each translation unit gets its own copy.
 */

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IImage.h>
#include <artec/sdk/base/Types.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace py   = pybind11;
namespace base = artec::sdk::base;

// UTF-8 std::string -> std::wstring (Windows API)
static std::wstring utf8_to_wcs(const std::string& str)
{
    if (str.empty()) return {};
    int len = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, nullptr, 0);
    if (len <= 1) return {};
    std::wstring out(len - 1, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, &out[0], len);
    return out;
}

// wchar_t* -> UTF-8 std::string (Windows API)
static std::string wcs_to_utf8(const wchar_t* wstr)
{
    if (!wstr || wstr[0] == L'\0') return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string out(len - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, &out[0], len, nullptr, nullptr);
    return out;
}

// ErrorCode -> throw std::runtime_error on failure
static void check_ec(base::ErrorCode ec, const char* context)
{
    if (ec != base::ErrorCode_OK)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "[ArtecSDK] %s failed (ErrorCode=0x%08X)",
            context, static_cast<unsigned>(ec));
        throw std::runtime_error(buf);
    }
}

// IImage -> (H, W, 3) uint8 RGB numpy array
static py::array_t<uint8_t> image_to_rgb(const base::IImage* img)
{
    int w     = img->getWidth();
    int h     = img->getHeight();
    int pitch = img->getPitch();
    base::PixelFormat fmt = img->getPixelFormat();
    const uint8_t* src = static_cast<const uint8_t*>(img->getPointer());

    auto arr = py::array_t<uint8_t>({(py::ssize_t)h, (py::ssize_t)w, (py::ssize_t)3});
    uint8_t* dst = arr.mutable_data();

    if (fmt == base::PixelFormat_BGR)
    {
        for (int r = 0; r < h; ++r)
        {
            const uint8_t* rs = src + r * pitch;
            uint8_t*       rd = dst + r * w * 3;
            for (int c = 0; c < w; ++c)
            {
                rd[c*3+0] = rs[c*3+2];
                rd[c*3+1] = rs[c*3+1];
                rd[c*3+2] = rs[c*3+0];
            }
        }
        return arr;
    }
    if (fmt == base::PixelFormat_RGB)
    {
        for (int r = 0; r < h; ++r)
            std::memcpy(dst + r * w * 3, src + r * pitch, w * 3);
        return arr;
    }
    if (fmt == base::PixelFormat_Mono)
    {
        for (int r = 0; r < h; ++r)
        {
            const uint8_t* rs = src + r * pitch;
            uint8_t*       rd = dst + r * w * 3;
            for (int c = 0; c < w; ++c)
                rd[c*3+0] = rd[c*3+1] = rd[c*3+2] = rs[c];
        }
        return arr;
    }

    return py::array_t<uint8_t>({(py::ssize_t)0, (py::ssize_t)0, (py::ssize_t)3});
}
