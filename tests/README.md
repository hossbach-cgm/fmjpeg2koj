# Decoder regression tests

The tests create their own JPEG-2000 frames with OpenJPEG. No patient data or external image fixtures are used.

Enable them with `-DBUILD_TESTING=ON`, build, then run:

```text
ctest --test-dir build-Release -C Release --output-on-failure
```

On Windows, the DCMTK and OpenJPEG DLL directories must be on `PATH`. The tests work with both static and shared fmjpeg2k builds.

## Consumer language mode

All public headers require C++20 or newer. An older consumer language mode produces an explicit `fmjpeg2k requires C++20 or newer` compiler error. MSVC uses `_MSVC_LANG` for this check, so `/Zc:__cplusplus` is not required.

The consumer must explicitly select its language mode (for example `set(CMAKE_CXX_STANDARD 20)` or `/std:c++20`). The library deliberately does not silently upgrade consumer targets through a public `cxx_std_20` compile feature.

When `BUILD_TESTING=ON`, CMake performs isolated compile checks using `tests/cxx_standard_probe.cc`: C++17 must fail with the library's diagnostic, and C++20 must succeed. These are configure-time checks, separate from the two runtime CTest tests.

## Sample width

`decode_sample_width` checks signed and unsigned samples in DICOM 8-/16-bit containers, including 8-bit precision in a 16-bit container, two-frame strides, short buffers, output boundaries, and rejection of unsupported RGB word output. MONOCHROME1 and PALETTE COLOR retain their interpretation and sample values. Repeated 17-bit grayscale rejection is exercised through the color-model query, frame decoding, and full decoding. These checks verify rejection and buffer safety; they are not a quantitative leak measurement.

## Color interpretation and DCMTK integration

`decode_color` exercises the registered codec through:

- `DcmPixelData::getDecompressedColorModel()` before decompression;
- `DcmPixelData::getUncompressedFrame()`;
- `DcmDataset::chooseRepresentation()`;
- `DicomImage`, with both complete decompression and partial frame access.

The supported color subset is unsigned, full-resolution, 8-bit RGB output:

| DICOM input interpretation | Codestream transform | Output |
| --- | --- | --- |
| RGB | No multi-component transform | RGB |
| YBR_RCT | Standard reversible MCT | RGB |
| YBR_ICT | Standard irreversible MCT | RGB |

OpenJPEG performs the inverse MCT; the codec does not apply it twice. Raw codestreams do not need a JP2 sRGB label. JP2 wrappers remain accepted when their codestream and decoded image satisfy the same checks.

Color COD markers in the main header and every tile-part header are checked. Component-specific COC coding and Part 2/custom MCT are deliberately rejected rather than interpreted speculatively. Other color interpretations, subsampling, signed color components, and word-sized RGB output remain unsupported.

The model query decodes the first frame into a temporary buffer to verify its answer, without modifying the compressed dataset. It therefore adds a first-frame decode before partial image processing. Subsequent frames are validated independently.

Frame-wise decoding leaves the compressed metadata unchanged and returns the output color model separately. After successful complete decompression, PhotometricInterpretation and PlanarConfiguration describe the output. A metadata change that invalidates the old representation is signaled to DCMTK through `removeOldRep`.

Coverage includes reversible and irreversible transforms, interleaved and planar data, fragmented multiframe inputs, multiple tiles, consistent and contradictory tile-level COD overrides, a final tile-part with unspecified length, contradictory DICOM interpretation, a mismatch in a later frame, and removal of stale compressed representations after a full color/layout change.

Forced layout options are tested for complete decompression. Partial `DicomImage` tests use the default restore-layout policy and dataset layouts 0 and 1; they do not establish support for forcing a layout different from the compressed dataset during partial image access.
