#pragma once
#include <stdint.h>
#include <stddef.h>
#include <WString.h>

// Convert a UTF-8 string to Latin-1 (ISO-8859-1) in place.
// Italian accented characters (à è é ì ò ù and uppercase) are in Unicode
// U+00C0–U+00FF, encoded in UTF-8 as two bytes: 0xC2/0xC3 + continuation.
// Characters outside the Latin-1 range (U+0100+) are replaced with '?'.
//
// dst may be the same pointer as src (conversion shrinks or keeps length).
// Returns the number of Latin-1 bytes written (excluding null terminator).
inline size_t utf8ToLatin1(const char* src, char* dst, size_t dstSize) {
    size_t i = 0, j = 0;
    while (src[i] && j < dstSize - 1) {
        uint8_t b = (uint8_t)src[i];
        if (b < 0x80) {
            // Plain ASCII — copy as-is
            dst[j++] = (char)b;
            i++;
        } else if ((b & 0xE0) == 0xC0) {
            // 2-byte UTF-8 sequence (U+0080 to U+07FF)
            uint8_t b2 = (uint8_t)src[i + 1];
            if ((b2 & 0xC0) == 0x80) {
                uint16_t cp = (uint16_t)((b & 0x1F) << 6) | (b2 & 0x3F);
                dst[j++] = (cp <= 0xFF) ? (char)cp : '?';
                i += 2;
            } else {
                dst[j++] = '?';
                i++;
            }
        } else {
            // 3- or 4-byte sequence — outside Latin-1, skip it
            dst[j++] = '?';
            i++;
            while (src[i] && ((uint8_t)src[i] & 0xC0) == 0x80) i++;
        }
    }
    dst[j] = '\0';
    return j;
}

// Arduino String convenience overload — returns a new Latin-1 String.
inline String utf8ToLatin1(const String& s) {
    // Latin-1 output is always <= input length (multi-byte → single byte)
    size_t maxLen = s.length() + 1;
    char* buf = new char[maxLen];
    utf8ToLatin1(s.c_str(), buf, maxLen);
    String result(buf);
    delete[] buf;
    return result;
}
