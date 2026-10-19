#include "dcmtk/config/osconfig.h"
#include "dcmtk/dcmdata/dctk.h"
#include "dcmtk/dcmdata/dcpixseq.h"
#include "dcmtk/dcmdata/dcpxitem.h"
#include "fmjpeg2k/djcodecd.h"
#include "fmjpeg2k/djcparam.h"
#include "fmjpeg2k/djerror.h"
#include "fmjpeg2k/memory_file.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

// Generate a lossless raw JPEG-2000 frame without external fixtures or tools.
static std::vector<Uint8> encode(const std::array<OPJ_INT32, 4>& values,
    unsigned precision, bool isSigned, unsigned components = 1)
{
    std::array<opj_image_cmptparm_t, 3> component = {};
    for (unsigned c = 0; c < components; ++c) {
        component[c].dx = component[c].dy = 1;
        component[c].w = component[c].h = 2;
        component[c].prec = precision;
        component[c].sgnd = isSigned;
    }
    opj_image_t* image = opj_image_create(components, component.data(),
        components == 1 ? OPJ_CLRSPC_GRAY : OPJ_CLRSPC_SRGB);
    require(image != nullptr, "Cannot create test image");
    image->x1 = image->y1 = 2;
    for (unsigned c = 0; c < components; ++c)
        std::copy(values.begin(), values.end(), image->comps[c].data);

    opj_cparameters_t parameters;
    opj_set_default_encoder_parameters(&parameters);
    parameters.tcp_numlayers = 1;
    parameters.tcp_rates[0] = 0;
    parameters.cp_disto_alloc = 1;
    parameters.numresolution = 1;
    parameters.irreversible = 0;

    std::vector<Uint8> encoded(65536);
    DecodeData output(encoded.data(), encoded.size());
    opj_stream_t* stream = opj_stream_create_memory_stream(&output, 4096, false);
    opj_codec_t* codec = opj_create_compress(OPJ_CODEC_J2K);
    const bool ok = stream && codec &&
        opj_setup_encoder(codec, &parameters, image) &&
        opj_start_compress(codec, image, stream) &&
        opj_encode(codec, stream) && opj_end_compress(codec, stream);
    if (stream) opj_stream_destroy(stream);
    if (codec) opj_destroy_codec(codec);
    opj_image_destroy(image);
    require(ok, "Cannot encode test frame");
    encoded.resize(output.offset);
    if (encoded.size() & 1) encoded.push_back(0);
    return encoded;
}

static void check(unsigned allocated, unsigned stored, bool isSigned)
{
    const int low = isSigned ? -(1 << (stored - 1)) : 0;
    const int high = isSigned ? (1 << (stored - 1)) - 1 : (1 << stored) - 1;
    const std::array<OPJ_INT32, 4> values = {low, low + 1, high - 1, high};
    const auto encoded = encode(values, stored, isSigned);

    DcmDataset dataset;
    require(dataset.putAndInsertUint16(DCM_Rows, 2).good(), "Rows");
    require(dataset.putAndInsertUint16(DCM_Columns, 2).good(), "Columns");
    dataset.putAndInsertUint16(DCM_SamplesPerPixel, 1);
    dataset.putAndInsertUint16(DCM_BitsAllocated, allocated);
    dataset.putAndInsertUint16(DCM_BitsStored, stored);
    dataset.putAndInsertUint16(DCM_HighBit, stored - 1);
    dataset.putAndInsertUint16(DCM_PixelRepresentation, isSigned ? 1 : 0);
    dataset.putAndInsertString(DCM_PhotometricInterpretation, "MONOCHROME2");
    dataset.putAndInsertString(DCM_NumberOfFrames, "2");

    DcmPixelSequence sequence(DCM_PixelSequenceTag);
    require(sequence.insert(new DcmPixelItem(DCM_PixelItemTag)).good(), "BOT");
    for (unsigned frame = 0; frame < 2; ++frame) {
        auto* item = new DcmPixelItem(DCM_PixelItemTag);
        require(item->putUint8Array(encoded.data(), encoded.size()).good(), "Fragment data");
        require(sequence.insert(item).good(), "Fragment");
    }

    const size_t frameSize = 4 * (allocated / 8);
    std::vector<Uint8> expected(frameSize);
    for (size_t i = 0; i < values.size(); ++i) {
        if (allocated == 8) expected[i] = static_cast<Uint8>(values[i]);
        else {
            const Uint16 value = static_cast<Uint16>(values[i]);
            std::memcpy(expected.data() + 2 * i, &value, sizeof(value));
        }
    }

    DJPEG2KDecoder decoder;
    DJPEG2KCodecParameter parameters;
    require(decoder.decodedBitsAllocated(allocated, stored) == allocated,
        "Incorrect advertised output width");

    // Word-aligned destination, with sentinel bytes after the requested frame.
    std::array<Uint16, 16> buffer;
    std::memset(buffer.data(), 0xAA, sizeof(buffer));
    auto* bytes = reinterpret_cast<Uint8*>(buffer.data());
    Uint32 fragment = 1;
    OFString color;
    require(decoder.decodeFrame(nullptr, &sequence, &parameters, &dataset, 0,
        fragment, buffer.data(), frameSize - 1, color).bad(), "Short buffer accepted");
    require(fragment == 1, "Short-buffer failure consumed a fragment");
    require(decoder.decodeFrame(nullptr, &sequence, &parameters, &dataset, 0,
        fragment, buffer.data(), frameSize, color).good(), "Frame decode failed");
    require(std::memcmp(bytes, expected.data(), frameSize) == 0, "Incorrect frame pixels");
    require(bytes[frameSize] == 0xAA, "Write past frame boundary");
    require(fragment == 2, "Incorrect next fragment");

    DcmPolymorphOBOW output(DCM_PixelData);
    DcmStack stack;
    stack.push(&dataset);
    stack.push(&output);
    require(decoder.decode(nullptr, &sequence, output, &parameters, stack).good(),
        "Full decode failed");
    require(output.getLength() == 2 * frameSize, "Incorrect full pixel data size");
    if (allocated == 8) {
        Uint8* data = nullptr;
        require(output.getUint8Array(data).good() && data, "Missing byte output");
        require(std::memcmp(data, expected.data(), frameSize) == 0, "Incorrect first frame");
        require(std::memcmp(data + frameSize, expected.data(), frameSize) == 0,
            "Incorrect second frame stride");
    }
    else {
        Uint16* data = nullptr;
        require(output.getUint16Array(data).good() && data, "Missing word output");
        require(std::memcmp(data, expected.data(), frameSize) == 0, "Incorrect first word frame");
        require(std::memcmp(reinterpret_cast<Uint8*>(data) + frameSize,
            expected.data(), frameSize) == 0, "Incorrect second word frame stride");
    }
    Uint16 actualAllocated = 0;
    dataset.findAndGetUint16(DCM_BitsAllocated, actualAllocated);
    require(actualAllocated == allocated, "BitsAllocated was changed");
    std::cout << "PASS: allocated=" << allocated << " stored=" << stored
        << " signed=" << isSigned << '\n';
}

static void checkUnsupportedRGBContainer()
{
    const auto encoded = encode({0, 1, 127, 255}, 8, false, 3);
    DcmDataset dataset;
    dataset.putAndInsertUint16(DCM_Rows, 2);
    dataset.putAndInsertUint16(DCM_Columns, 2);
    dataset.putAndInsertUint16(DCM_SamplesPerPixel, 3);
    dataset.putAndInsertUint16(DCM_BitsAllocated, 16);
    dataset.putAndInsertUint16(DCM_BitsStored, 8);
    dataset.putAndInsertUint16(DCM_HighBit, 7);
    dataset.putAndInsertUint16(DCM_PixelRepresentation, 0);
    dataset.putAndInsertString(DCM_PhotometricInterpretation, "RGB");
    dataset.putAndInsertUint16(DCM_PlanarConfiguration, 0);
    DcmPixelSequence sequence(DCM_PixelSequenceTag);
    sequence.insert(new DcmPixelItem(DCM_PixelItemTag));
    auto* item = new DcmPixelItem(DCM_PixelItemTag);
    require(item->putUint8Array(encoded.data(), encoded.size()).good(), "RGB fragment");
    sequence.insert(item);
    DJPEG2KDecoder decoder;
    DJPEG2KCodecParameter parameters;
    std::array<Uint16, 12> buffer;
    buffer.fill(0xAAAA);
    Uint32 fragment = 1;
    OFString color;
    require(decoder.decodeFrame(nullptr, &sequence, &parameters, &dataset, 0,
        fragment, buffer.data(), sizeof(buffer), color) == EC_J2KUnsupportedBitDepth,
        "Unsupported word-sized RGB output must be rejected");
    for (Uint16 value : buffer) require(value == 0xAAAA, "Rejected RGB wrote output");
    require(fragment == 1, "Rejected RGB consumed a fragment");
}

int main()
{
    try {
        for (bool isSigned : {false, true}) {
            check(8, 8, isSigned);
            check(16, 8, isSigned);
            check(16, 12, isSigned);
            check(16, 16, isSigned);
        }
        checkUnsupportedRGBContainer();
        DJPEG2KDecoder decoder;
        for (unsigned allocated : {0, 1, 7, 12, 24, 32})
            require(decoder.decodedBitsAllocated(allocated, 1) == 0,
                "Unsupported container advertised");
        require(decoder.decodedBitsAllocated(8, 0) == 0, "Zero precision accepted");
        require(decoder.decodedBitsAllocated(8, 9) == 0, "Precision exceeds container");
        require(decoder.decodedBitsAllocated(16, 17) == 0, "Unsupported precision accepted");
    }
    catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
