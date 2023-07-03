
/*******************************************************************************

MIT License

Copyright (c) dynamic-static

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include "dynamic-static/filesystem.hpp"

#include <fstream>

namespace dst {

void read_bytes(const std::filesystem::path& filePath, std::vector<uint8_t>& bytes)
{
    bytes.clear();
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (file.is_open()) {
        bytes.resize(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read((char*)bytes.data(), bytes.size());
    }
}

std::vector<uint8_t> read_bytes(const std::filesystem::path& filePath)
{
    std::vector<uint8_t> bytes;
    read_bytes(filePath, bytes);
    return bytes;
}

} // namespace dst
