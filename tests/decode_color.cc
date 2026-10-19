#include "dcmtk/config/osconfig.h"
#include "dcmtk/dcmdata/dctk.h"
#include "dcmtk/dcmdata/dcpixseq.h"
#include "dcmtk/dcmdata/dcpxitem.h"
#include "dcmtk/dcmimgle/dcmimage.h"
#include "dcmtk/dcmimage/diregist.h"
#include "fmjpeg2k/djdecode.h"
#include "fmjpeg2k/memory_file.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

static constexpr size_t width = 4, height = 4, pixels = width * height;
static void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

static std::vector<Uint8> referencePixels()
{
    std::vector<Uint8> result(3 * pixels);
    for (size_t i = 0; i < pixels; ++i) {
        result[3 * i] = static_cast<Uint8>(20 + i * 11);
        result[3 * i + 1] = static_cast<Uint8>(210 - i * 7);
        result[3 * i + 2] = static_cast<Uint8>(40 + i * 5);
    }
    return result;
}

static std::vector<Uint8> encode(bool mct, bool irreversible, bool jp2 = false)
{
    std::array<opj_image_cmptparm_t, 3> components = {};
    for (auto& c : components) {
        c.dx = c.dy = 1;
        c.w = width; c.h = height; c.prec = 8;
    }
    opj_image_t* image = opj_image_create(3, components.data(), OPJ_CLRSPC_SRGB);
    require(image != nullptr, "Test image allocation failed");
    image->x1 = width; image->y1 = height;
    const auto rgb = referencePixels();
    for (size_t i = 0; i < pixels; ++i)
        for (size_t c = 0; c < 3; ++c) image->comps[c].data[i] = rgb[3 * i + c];

    opj_cparameters_t parameters;
    opj_set_default_encoder_parameters(&parameters);
    parameters.tcp_numlayers = 1;
    parameters.tcp_rates[0] = 0;
    parameters.cp_disto_alloc = 1;
    parameters.numresolution = 1;
    parameters.tcp_mct = mct ? 1 : 0;
    parameters.irreversible = irreversible ? 1 : 0;
    // Exercise multiple tile-parts, not just a single COD/header and payload.
    parameters.tile_size_on = OPJ_TRUE;
    parameters.cp_tdx = parameters.cp_tdy = 2;

    std::vector<Uint8> encoded(65536);
    DecodeData output(encoded.data(), encoded.size());
    opj_stream_t* stream = opj_stream_create_memory_stream(&output, 4096, false);
    opj_codec_t* codec = opj_create_compress(jp2 ? OPJ_CODEC_JP2 : OPJ_CODEC_J2K);
    const bool ok = stream && codec && opj_setup_encoder(codec, &parameters, image) &&
        opj_start_compress(codec, image, stream) && opj_encode(codec, stream) &&
        opj_end_compress(codec, stream);
    if (stream) opj_stream_destroy(stream);
    if (codec) opj_destroy_codec(codec);
    opj_image_destroy(image);
    require(ok, "Test encoding failed");
    encoded.resize(output.offset);
    if (encoded.size() & 1) encoded.push_back(0);
    return encoded;
}

struct Fixture {
    DcmDataset dataset;
    DcmPixelData* pixel;
    E_TransferSyntax syntax;
    Fixture(const std::vector<Uint8>& encoded, const char* pi, bool lossy, Uint16 planar = 0)
        : pixel(new DcmPixelData(DCM_PixelData)),
          syntax(lossy ? EXS_JPEG2000 : EXS_JPEG2000LosslessOnly)
    {
        dataset.putAndInsertString(DCM_SOPClassUID, UID_SecondaryCaptureImageStorage);
        dataset.putAndInsertString(DCM_SOPInstanceUID, "1.2.826.0.1.3680043.10.999.1");
        dataset.putAndInsertUint16(DCM_Rows, height);
        dataset.putAndInsertUint16(DCM_Columns, width);
        dataset.putAndInsertUint16(DCM_SamplesPerPixel, 3);
        dataset.putAndInsertUint16(DCM_BitsAllocated, 8);
        dataset.putAndInsertUint16(DCM_BitsStored, 8);
        dataset.putAndInsertUint16(DCM_HighBit, 7);
        dataset.putAndInsertUint16(DCM_PixelRepresentation, 0);
        dataset.putAndInsertUint16(DCM_PlanarConfiguration, planar);
        dataset.putAndInsertString(DCM_PhotometricInterpretation, pi);
        dataset.putAndInsertString(DCM_NumberOfFrames, "2");
        auto* sequence = new DcmPixelSequence(DCM_PixelSequenceTag);
        auto* bot = new DcmPixelItem(DCM_PixelItemTag);
        sequence->insert(bot);
        // Two fragments per frame and a valid Basic Offset Table.
        const std::array<Uint32, 2> offsets = {0, static_cast<Uint32>(encoded.size() + 16)};
        bot->putUint8Array(reinterpret_cast<const Uint8*>(offsets.data()), 8);
        const size_t split = (encoded.size() / 2) & ~size_t(1);
        for (unsigned frame = 0; frame < 2; ++frame) {
            for (unsigned part = 0; part < 2; ++part) {
                auto* item = new DcmPixelItem(DCM_PixelItemTag);
                const size_t offset = part == 0 ? 0 : split;
                const size_t length = part == 0 ? split : encoded.size() - split;
                require(item->putUint8Array(encoded.data() + offset, length).good(), "Fragment creation failed");
                sequence->insert(item);
            }
        }
        pixel->putOriginalRepresentation(syntax, nullptr, sequence);
        require(dataset.insert(pixel).good(), "Pixel insertion failed");
    }
    OFString pi() {
        OFString result;
        dataset.findAndGetOFString(DCM_PhotometricInterpretation, result);
        return result;
    }
    Uint16 planar() {
        Uint16 result = 2;
        dataset.findAndGetUint16(DCM_PlanarConfiguration, result);
        return result;
    }
};

static void checkPixels(const Uint8* actual, const std::vector<Uint8>& expected, int tolerance)
{
    require(actual != nullptr, "Missing pixel buffer");
    for (size_t i = 0; i < expected.size(); ++i)
        require(std::abs(int(actual[i]) - int(expected[i])) <= tolerance, "Incorrect RGB pixel value");
}

static void checkPaths(const char* pi, bool mct, bool irreversible, Uint16 planar = 0,
    bool jp2 = false, const std::vector<Uint8>* supplied = nullptr)
{
    const auto encoded = supplied ? *supplied : encode(mct, irreversible, jp2);
    auto expected = referencePixels();
    if (planar) {
        const auto interleaved = expected;
        for (size_t c = 0; c < 3; ++c)
            for (size_t i = 0; i < pixels; ++i) expected[c * pixels + i] = interleaved[3 * i + c];
    }
    const int tolerance = irreversible ? 2 : 0;
    Fixture frame(encoded, pi, irreversible, planar);
    OFString color;
    require(frame.pixel->getDecompressedColorModel(&frame.dataset, color).good(), "Pre-decode model query failed");
    require(color == "RGB", "Pre-decode model is not RGB");
    require(frame.pi() == pi && frame.planar() == planar, "Model query modified metadata");
    require(frame.pixel->hasRepresentation(frame.syntax, nullptr), "Model query removed compressed data");

    std::array<Uint16, 3 * pixels / 2 + 1> words;
    std::memset(words.data(), 0xAA, sizeof(words));
    auto* bytes = reinterpret_cast<Uint8*>(words.data());
    Uint32 fragment = 0;
    for (unsigned i = 0; i < 2; ++i) {
        require(frame.pixel->getUncompressedFrame(&frame.dataset, i, fragment,
            words.data(), 3 * pixels, color).good(), "DCMTK frame decode failed");
        require(color == "RGB", "Frame model is not RGB");
        checkPixels(bytes, expected, tolerance);
        require(bytes[3 * pixels] == 0xAA, "Frame output crossed buffer boundary");
        require(frame.pi() == pi && frame.planar() == planar, "Frame decode modified metadata");
    }
    require(fragment == 5, "Incorrect next fragment after two frames");

    Fixture full(encoded, pi, irreversible, planar);
    require(full.dataset.chooseRepresentation(EXS_LittleEndianExplicit, nullptr).good(), "chooseRepresentation failed");
    require(full.pi() == "RGB" && full.planar() == planar, "Incorrect full decode metadata");
    require(full.pixel->getDecompressedColorModel(&full.dataset, color).good() && color == "RGB", "Full decode model mismatch");
    Uint16* data = nullptr;
    require(full.pixel->getUint16Array(data).good(), "Cannot access full pixel data");
    require(full.pixel->getLength() == 6 * pixels, "Incorrect full output length");
    checkPixels(reinterpret_cast<Uint8*>(data), expected, tolerance);
    checkPixels(reinterpret_cast<Uint8*>(data) + 3 * pixels, expected, tolerance);
    if (OFString(pi) != "RGB")
        require(!full.pixel->hasRepresentation(full.syntax, nullptr), "Stale compressed representation retained");

    for (unsigned long flags : {0ul, CIF_UsePartialAccessToPixelData}) {
        Fixture rendering(encoded, pi, irreversible, planar);
        DicomImage image(&rendering.dataset, rendering.syntax, flags, 0, 1);
        require(image.getStatus() == EIS_Normal, "DicomImage construction failed");
        require(image.getPhotometricInterpretation() == EPI_RGB, "DicomImage selected wrong interpretation");
        checkPixels(static_cast<const Uint8*>(image.getOutputData(8)), referencePixels(), tolerance);
        if (flags) {
            require(rendering.pi() == pi, "Partial DicomImage modified PI");
            require(image.processNextFrames(1) != 0, "Partial DicomImage next frame failed");
            checkPixels(static_cast<const Uint8*>(image.getOutputData(8)), referencePixels(), tolerance);
        }
    }
    std::cout << "PASS DCMTK paths: " << pi << " planar=" << planar << " JP2=" << jp2 << '\n';
}

static void checkRejected(const std::vector<Uint8>& encoded, const char* pi)
{
    Fixture fixture(encoded, pi, true);
    OFString color;
    require(fixture.pixel->getDecompressedColorModel(&fixture.dataset, color).bad(), "Unsupported color query succeeded");
    std::array<Uint16, 3 * pixels / 2> buffer;
    buffer.fill(0xAAAA);
    Uint32 fragment = 0;
    require(fixture.pixel->getUncompressedFrame(&fixture.dataset, 0, fragment,
        buffer.data(), sizeof(buffer), color).bad(), "Unsupported color frame succeeded");
    for (auto value : buffer) require(value == 0xAAAA, "Rejected frame wrote output");
    require(fixture.dataset.chooseRepresentation(EXS_LittleEndianExplicit, nullptr).bad(), "Unsupported full decode succeeded");
    require(fixture.pi() == pi && fixture.planar() == 0, "Rejected decode modified color metadata");
}

static void checkLateFrameMismatch()
{
    Fixture fixture(encode(true, false), "YBR_RCT", false);
    OFString color;
    require(fixture.pixel->getDecompressedColorModel(&fixture.dataset, color).good(), "First-frame model query failed");
    DcmPixelSequence* sequence = nullptr;
    require(fixture.pixel->getEncapsulatedRepresentation(fixture.syntax, nullptr, sequence).good(), "Missing compressed sequence");
    const auto badFrame = encode(false, false);
    const size_t split = (badFrame.size() / 2) & ~size_t(1);
    for (unsigned part = 0; part < 2; ++part) {
        DcmPixelItem* item = nullptr;
        require(sequence->getItem(item, 3 + part).good(), "Missing second-frame fragment");
        const size_t offset = part ? split : 0;
        require(item->putUint8Array(badFrame.data() + offset,
            part ? badFrame.size() - split : split).good(), "Cannot replace second frame");
    }
    require(fixture.dataset.chooseRepresentation(EXS_LittleEndianExplicit, nullptr).bad(), "Inconsistent later frame accepted");
    require(fixture.pi() == "YBR_RCT", "Failed full decode changed PI");
    require(fixture.pixel->hasRepresentation(fixture.syntax, nullptr), "Failed decode removed original representation");
}

static void checkForcedFullLayout()
{
    FMJPEG2KDecoderRegistration::cleanup();
    FMJPEG2KDecoderRegistration::registerCodecs(EJ2KUC_default, EJ2KPC_colorByPlane);
    Fixture fixture(encode(false, false), "RGB", false);
    require(fixture.dataset.chooseRepresentation(EXS_LittleEndianExplicit, nullptr).good(), "Forced-planar full decode failed");
    require(fixture.planar() == 1, "Full decode did not update PlanarConfiguration");
    require(!fixture.pixel->hasRepresentation(fixture.syntax, nullptr), "Old layout representation retained");
    Uint16* data = nullptr;
    require(fixture.pixel->getUint16Array(data).good(), "Missing forced-planar output");
    auto expected = referencePixels();
    const auto interleaved = expected;
    for (size_t c = 0; c < 3; ++c)
        for (size_t i = 0; i < pixels; ++i) expected[c * pixels + i] = interleaved[3 * i + c];
    checkPixels(reinterpret_cast<Uint8*>(data), expected, 0);
    FMJPEG2KDecoderRegistration::cleanup();
    FMJPEG2KDecoderRegistration::registerCodecs();
}

int main()
{
    FMJPEG2KDecoderRegistration::registerCodecs();
    try {
        checkPaths("RGB", false, false);
        checkPaths("YBR_RCT", true, false);
        checkPaths("YBR_ICT", true, true);
        checkPaths("RGB", false, false, 1);
        checkPaths("YBR_RCT", true, false, 1);
        checkPaths("RGB", false, false, 0, true);
        checkRejected(encode(false, false), "YBR_FULL");
        checkRejected(encode(false, false), "YBR_RCT");
        checkRejected(encode(true, false), "RGB");
        checkRejected(encode(true, false), "YBR_ICT");
        checkRejected(encode(true, true), "YBR_RCT");
        // A contradictory COD in a tile header must not bypass the main-header check.
        auto overrideCOD = encode(true, false);
        size_t mainCOD = 0, firstSOT = 0;
        for (size_t i = 2; i + 14 <= overrideCOD.size(); ++i) {
            if (!mainCOD && overrideCOD[i] == 0xff && overrideCOD[i + 1] == 0x52) mainCOD = i;
            if (overrideCOD[i] == 0xff && overrideCOD[i + 1] == 0x90) { firstSOT = i; break; }
        }
        require(mainCOD && firstSOT, "Missing coding markers in test stream");
        const size_t codLength = 2 + (size_t(overrideCOD[mainCOD + 2]) << 8) + overrideCOD[mainCOD + 3];
        std::vector<Uint8> tileCOD(overrideCOD.begin() + mainCOD, overrideCOD.begin() + mainCOD + codLength);
        tileCOD[8] = 0; // disable MCT only in first tile
        Uint32 psot = 0;
        for (size_t i = 0; i < 4; ++i) psot = (psot << 8) | overrideCOD[firstSOT + 6 + i];
        psot += static_cast<Uint32>(codLength);
        for (int i = 3; i >= 0; --i) { overrideCOD[firstSOT + 6 + i] = psot & 255; psot >>= 8; }
        overrideCOD.insert(overrideCOD.begin() + firstSOT + 12, tileCOD.begin(), tileCOD.end());
        checkRejected(overrideCOD, "YBR_RCT");
        // A matching tile-level COD is legal and must still decode correctly.
        overrideCOD[firstSOT + 12 + 8] = 1;
        checkPaths("YBR_RCT", true, false, 0, false, &overrideCOD);
        auto unspecifiedLength = encode(true, false);
        size_t lastSOT = 0;
        for (size_t i = 2; i + 12 <= unspecifiedLength.size(); ++i)
            if (unspecifiedLength[i] == 0xff && unspecifiedLength[i + 1] == 0x90) lastSOT = i;
        require(lastSOT != 0, "Missing final tile-part");
        std::fill(unspecifiedLength.begin() + lastSOT + 6, unspecifiedLength.begin() + lastSOT + 10, 0);
        checkPaths("YBR_RCT", true, false, 0, false, &unspecifiedLength);
        checkLateFrameMismatch();
        checkForcedFullLayout();
    }
    catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        FMJPEG2KDecoderRegistration::cleanup();
        return 1;
    }
    FMJPEG2KDecoderRegistration::cleanup();
    return 0;
}
