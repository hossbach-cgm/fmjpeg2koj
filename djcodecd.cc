/*
 *
 *  Copyright 2015-2017 Ing-Long Eric Kuo
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *
 *  Module:  fmjpeg2k
 *
 *  Author:  Ing-Long Eric Kuo
 *
 *  Purpose: codec classes for JPEG-2000 decoders.
 *
 */

#include "dcmtk/config/osconfig.h"
#include "fmjpeg2k/djcodecd.h"

#include "dcmtk/ofstd/ofstream.h"    /* for ofstream */
#include "dcmtk/ofstd/ofcast.h"      /* for casts */
#include "dcmtk/ofstd/offile.h"      /* for class OFFile */
#include "dcmtk/ofstd/ofstd.h"       /* for class OFStandard */
#include "dcmtk/dcmdata/dcdatset.h"  /* for class DcmDataset */
#include "dcmtk/dcmdata/dcdeftag.h"  /* for tag constants */
#include "dcmtk/dcmdata/dcpixseq.h"  /* for class DcmPixelSequence */
#include "dcmtk/dcmdata/dcpxitem.h"  /* for class DcmPixelItem */
#include "dcmtk/dcmdata/dcvrpobw.h"  /* for class DcmPolymorphOBOW */
#include "dcmtk/dcmdata/dcswap.h"    /* for swapIfNecessary() */
#include "dcmtk/dcmdata/dcuid.h"     /* for dcmGenerateUniqueIdentifer()*/
#include "fmjpeg2k/djcparam.h"  /* for class DJP2KCodecParameter */
#include "fmjpeg2k/djerror.h"                 /* for private class DJLSError */

 // JPEG-2000 library (OpenJPEG) includes
#include "openjpeg.h"

#include "fmjpeg2k/memory_file.h"





DJPEG2KDecoderBase::DJPEG2KDecoderBase()
    : DcmCodec()
{
}


DJPEG2KDecoderBase::~DJPEG2KDecoderBase()
{
}


OFBool DJPEG2KDecoderBase::canChangeCoding(
    const E_TransferSyntax oldRepType,
    const E_TransferSyntax newRepType) const
{
    // this codec only handles conversion from JPEG-2000 to uncompressed.

    const bool oldIsJ2K =
        oldRepType == EXS_JPEG2000LosslessOnly ||
        oldRepType == EXS_JPEG2000 ||
        oldRepType == EXS_JPEG2000MulticomponentLosslessOnly ||
        oldRepType == EXS_JPEG2000Multicomponent;

    const bool newIsUncompressed =
        newRepType == EXS_LittleEndianExplicit ||
        newRepType == EXS_LittleEndianImplicit;

    return (oldIsJ2K && newIsUncompressed) ? OFTrue : OFFalse;
}


OFCondition DJPEG2KDecoderBase::decode(
    const DcmRepresentationParameter* /* fromRepParam */,
    DcmPixelSequence* pixSeq,
    DcmPolymorphOBOW& uncompressedPixelData,
    const DcmCodecParameter* codecparam,
    const DcmStack& objStack) const
{
    if (!pixSeq)
    {
        FMJPEG2K_DEBUG("DJPEG2KDecoderBase::decode: Invalid pixel sequence");
        return EC_IllegalParameter;
    }

    // retrieve pointer to dataset from parameter stack
    DcmStack localStack(objStack);
    (void)localStack.pop();  // pop pixel data element from stack
    DcmObject* dobject = localStack.pop(); // this is the item in which the pixel data is located
    if ((!dobject) || ((dobject->ident() != EVR_dataset) && (dobject->ident() != EVR_item))) return EC_InvalidTag;
    DcmItem* dataset = OFstatic_cast(DcmItem*, dobject);
    OFBool numberOfFramesPresent = OFFalse;

    // determine properties of uncompressed dataset
    Uint16 imageSamplesPerPixel = 0;
    if (dataset->findAndGetUint16(DCM_SamplesPerPixel, imageSamplesPerPixel).bad()) return EC_TagNotFound;
    // we only handle one or three samples per pixel
    if ((imageSamplesPerPixel != 3) && (imageSamplesPerPixel != 1)) return EC_InvalidTag;

    Uint16 imageRows = 0;
    if (dataset->findAndGetUint16(DCM_Rows, imageRows).bad()) return EC_TagNotFound;
    if (imageRows < 1) return EC_InvalidTag;

    Uint16 imageColumns = 0;
    if (dataset->findAndGetUint16(DCM_Columns, imageColumns).bad()) return EC_TagNotFound;
    if (imageColumns < 1) return EC_InvalidTag;

    // number of frames is an optional attribute - we don't mind if it isn't present.
    Sint32 imageFrames = 0;
    if (dataset->findAndGetSint32(DCM_NumberOfFrames, imageFrames).good()) numberOfFramesPresent = OFTrue;

    // must have at least BOT (item 0) + >=1 fragment
    const Uint32 numItems = pixSeq->card();
    if (numItems < 2) 
    {
        FMJPEG2K_WARN("JPEG-2000 pixel sequence contains no image data");
        return EC_CorruptedData;
    }

    if (imageFrames >= OFstatic_cast(Sint32, numItems))
        imageFrames = numItems - 1; // limit to available fragment range minus BOT
    if (imageFrames < 1)
        imageFrames = 1;

    Uint16 imageBitsStored = 0;
    if (dataset->findAndGetUint16(DCM_BitsStored, imageBitsStored).bad()) return EC_TagNotFound;

    Uint16 imageBitsAllocated = 0;
    if (dataset->findAndGetUint16(DCM_BitsAllocated, imageBitsAllocated).bad()) return EC_TagNotFound;

    Uint16 imageHighBit = 0;
    if (dataset->findAndGetUint16(DCM_HighBit, imageHighBit).bad()) return EC_TagNotFound;

    // sanity checks
    if (imageBitsStored < 1 || imageBitsStored > imageBitsAllocated) return EC_InvalidTag;
    if (imageHighBit >= imageBitsAllocated) return EC_InvalidTag;

    //we only support up to 16 bits per sample
    if ((imageBitsStored < 1) || (imageBitsStored > 16)) return EC_J2KUnsupportedBitDepth;


    // determine the number of bytes per sample (bits allocated) for the de-compressed object.
    const Uint16 bytesPerSample = (imageBitsAllocated > 8) ? 2 : 1;

    // 64-bit math to avoid overflow
    const uint64_t pixels = static_cast<uint64_t>(imageRows) * static_cast<uint64_t>(imageColumns);
    if (pixels == 0 || pixels > (uint64_t(1) << 31))  // simple sanity cap
    {
        FMJPEG2K_WARN("Decompressed image size is unreasonably large");
        return EC_MemoryExhausted;
    }

    const uint64_t bytesPerFrame = static_cast<uint64_t>(bytesPerSample) * pixels * static_cast<uint64_t>(imageSamplesPerPixel);

    // guard against huge allocations
    if (bytesPerFrame > std::numeric_limits<Uint32>::max() ||
        bytesPerFrame * uint64_t(imageFrames) > std::numeric_limits<Uint32>::max())
    {
        FMJPEG2K_WARN("Decompressed image size is too large to be processed");
        return EC_MemoryExhausted;
    }

    // total size (all frames), aligned to even length (OB/OW rule)
    uint64_t total64 = bytesPerFrame * static_cast<uint64_t>(imageFrames);
    total64 += (total64 & 1ull); // if odd, add one byte

    const Uint32 frameSize = static_cast<Uint32>(bytesPerFrame);
    const Uint32 totalSize = static_cast<Uint32>(total64);
    
    // sanity (should be even by construction)
    if (totalSize & 1u) 
    {
        return EC_CorruptedData;
    }

    // assume we can cast the codec parameter to what we need
    const DJPEG2KCodecParameter* djcp = OFreinterpret_cast(const DJPEG2KCodecParameter*, codecparam);

    // determine planar configuration for uncompressed data
    OFString imageSopClass;
    OFString imagePhotometricInterpretation;
    dataset->findAndGetOFString(DCM_SOPClassUID, imageSopClass);
    dataset->findAndGetOFString(DCM_PhotometricInterpretation, imagePhotometricInterpretation);

    // allocate space for uncompressed pixel data element
    Uint16* pixeldata16 = NULL;
    OFCondition result = uncompressedPixelData.createUint16Array(totalSize / sizeof(Uint16), pixeldata16);
    if (result.bad()) return result;

    Uint8* pixeldata8 = OFreinterpret_cast(Uint8*, pixeldata16);
    Sint32 currentFrame = 0;
    Uint32 currentItem = 1; // item 0 contains the offset table
    OFBool done = OFFalse;

    while (result.good() && !done)
    {
        FMJPEG2K_DEBUG("JPEG-2000 decoder processes frame " << (currentFrame + 1));

        result = decodeFrame(pixSeq, djcp, dataset, currentFrame, currentItem, pixeldata8, frameSize,
            imageFrames, imageColumns, imageRows, imageSamplesPerPixel, bytesPerSample);

        if (result.good())
        {
            // increment frame number, check if we're finished
            if (++currentFrame == imageFrames) done = OFTrue;
            pixeldata8 += frameSize;
        }
    }

    if (currentFrame == 0)
    {
        // nothing could be decoded at all
        FMJPEG2K_WARN("No JPEG-2000 frames could be decoded");
        return EC_CorruptedData;
    }

    // mismatch is common in the wild; do not fail hard
    if (result.good() && (numberOfFramesPresent && imageFrames != currentFrame)) 
    {
        FMJPEG2K_WARN("NumberOfFrames mismatch: tag=" << imageFrames
            << " decoded=" << currentFrame << " (updating tag)");
    }

    if (result.good() && (numberOfFramesPresent || (currentFrame > 1))) {
        char numBuf[20];
        OFStandard::snprintf(numBuf, sizeof(numBuf), "%ld", static_cast<long>(currentFrame));
        result = dataset->putAndInsertString(DCM_NumberOfFrames, numBuf);
    }

    if (result.good())
    {
        // the following operations do not affect the Image Pixel Module
        // but other modules such as SOP Common.  We only perform these
        // changes if we're on the main level of the dataset,
        // which should always identify itself as dataset, not as item.
        if ((dataset->ident() == EVR_dataset) && djcp &&(djcp->getUIDCreation() == EJ2KUC_always))
        {
            // create new SOP instance UID
            result = DcmCodec::newInstance((DcmItem*)dataset, NULL, NULL, NULL);
        }
    }

    return result;
}


OFCondition DJPEG2KDecoderBase::decode(
    const DcmRepresentationParameter* fromRepParam,
    DcmPixelSequence* pixSeq,
    DcmPolymorphOBOW& uncompressedPixelData,
    const DcmCodecParameter* cp,
    const DcmStack& objStack,
    OFBool& removeOldRep) const
{
    // removeOldRep is left as it is, pixel data in original DICOM dataset is not modified
    return decode(fromRepParam, pixSeq, uncompressedPixelData, cp, objStack);
}


OFCondition DJPEG2KDecoderBase::decodeFrame(
    const DcmRepresentationParameter* /* fromParam */,
    DcmPixelSequence* fromPixSeq,
    const DcmCodecParameter* cp,
    DcmItem* dataset,
    Uint32 frameNo,
    Uint32& currentItem,
    void* buffer,
    Uint32 bufSize,
    OFString& decompressedColorModel) const
{
    OFCondition result = EC_Normal;

    if (!fromPixSeq || !buffer)
    {
        return EC_IllegalParameter;
    }

    const Uint32 numItems = fromPixSeq->card();
    if (numItems < 2)
    {
        FMJPEG2K_WARN("JPEG-2000 pixel sequence contains no image data");
        return EC_CorruptedData;
    }

    // assume we can cast the codec parameter to what we need
    const DJPEG2KCodecParameter* djcp = OFreinterpret_cast(const DJPEG2KCodecParameter*, cp);

    // determine properties of uncompressed dataset
    Uint16 imageSamplesPerPixel = 0;
    if (dataset->findAndGetUint16(DCM_SamplesPerPixel, imageSamplesPerPixel).bad()) return EC_TagNotFound;
    // we only handle one or three samples per pixel
    if ((imageSamplesPerPixel != 3) && (imageSamplesPerPixel != 1)) return EC_InvalidTag;

    Uint16 imageRows = 0;
    if (dataset->findAndGetUint16(DCM_Rows, imageRows).bad()) return EC_TagNotFound;
    if (imageRows < 1) return EC_InvalidTag;

    Uint16 imageColumns = 0;
    if (dataset->findAndGetUint16(DCM_Columns, imageColumns).bad()) return EC_TagNotFound;
    if (imageColumns < 1) return EC_InvalidTag;

    Uint16 imageBitsStored = 0;
    if (dataset->findAndGetUint16(DCM_BitsStored, imageBitsStored).bad()) return EC_TagNotFound;

    Uint16 imageBitsAllocated = 0;
    if (dataset->findAndGetUint16(DCM_BitsAllocated, imageBitsAllocated).bad()) return EC_TagNotFound;

    //we only support up to 16 bits per sample
    if ((imageBitsStored < 1) || (imageBitsStored > 16)) return EC_J2KUnsupportedBitDepth;

    const Uint16 bytesPerSample = (imageBitsAllocated > 8) ? 2 : 1;

    const uint64_t pixels = uint64_t(imageRows) * uint64_t(imageColumns);
    const uint64_t bytesPerFrame = uint64_t(bytesPerSample) * pixels * uint64_t(imageSamplesPerPixel);
    if (bytesPerFrame == 0 || bytesPerFrame > std::numeric_limits<Uint32>::max())
    {
        FMJPEG2K_WARN("Decompressed image size is unreasonably large");
        return EC_MemoryExhausted;
    }
    if (bytesPerFrame > bufSize)
    {
        return EC_InvalidStream; // buffer too small
    }

    Sint32 imageFrames = 0;
    (void)dataset->findAndGetSint32(DCM_NumberOfFrames, imageFrames);
    if (imageFrames >= OFstatic_cast(Sint32, numItems)) 
    {
        imageFrames = numItems - 1;
    }
    if (imageFrames < 1) 
    {
        imageFrames = 1;
    }

    if (currentItem == 0) {
        result = determineStartFragment(frameNo, imageFrames, fromPixSeq, currentItem);
        if (result.bad()) return result;
        if (currentItem == 0) return EC_CorruptedData; // must not be BOT
    }

    if (result.good())
    {
        // We got all the data we need from the dataset, let's start decoding
        FMJPEG2K_DEBUG("Starting to decode frame " << frameNo << " with fragment " << currentItem);
        result = decodeFrame(fromPixSeq, djcp, dataset, frameNo, currentItem, buffer, bufSize,
            imageFrames, imageColumns, imageRows, imageSamplesPerPixel, bytesPerSample);
    }

    auto isValidPI = [](const OFString& s) {
        static const char* k[] = {
            "MONOCHROME1","MONOCHROME2","RGB","PALETTE COLOR",
            "YBR_FULL","YBR_FULL_422","YBR_PARTIAL_422","YBR_ICT","YBR_RCT"
        };
        for (auto v : k) if (s == v) return true;
        return false;
        };

    OFString pi;
    if (dataset->findAndGetOFString(DCM_PhotometricInterpretation, pi).good() && isValidPI(pi)) {
        decompressedColorModel = pi;
    }
    else {
        // infer a sane local default without modifying the dataset
        const OFBool hasPalette =
            dataset->tagExistsWithValue(DCM_RedPaletteColorLookupTableData) ||
            dataset->tagExistsWithValue(DCM_GreenPaletteColorLookupTableData) ||
            dataset->tagExistsWithValue(DCM_BluePaletteColorLookupTableData);

        if (hasPalette)          decompressedColorModel = "PALETTE COLOR";
        else if (imageSamplesPerPixel == 3) decompressedColorModel = "RGB";
        else                      decompressedColorModel = "MONOCHROME2";

        FMJPEG2K_WARN("PI missing/invalid; using inferred color model: " << decompressedColorModel);
    }

    return result;
}

OFCondition copyUint32ToUint8(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows);

OFCondition copyUint32ToUint16(
    opj_image_t* image,
    Uint16* imageFrame,
    Uint16 columns,
    Uint16 rows);

OFCondition copyRGBUint8ToRGBUint8(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows);

OFCondition copyRGBUint8ToRGBUint8Planar(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows);

OFCondition DJPEG2KDecoderBase::decodeFrame(
    DcmPixelSequence* fromPixSeq,
    const DJPEG2KCodecParameter* cp,
    DcmItem* dataset,
    Uint32 frameNo,
    Uint32& currentItem,
    void* buffer,
    Uint32 bufSize,
    Sint32 imageFrames,
    Uint16 imageColumns,
    Uint16 imageRows,
    Uint16 imageSamplesPerPixel,
    Uint16 bytesPerSample)
{
    if (!cp)
    {
        return EC_IllegalParameter;
    }

    DcmPixelItem* pixItem = NULL;
    Uint8* jlsData = NULL;
    Uint8* jlsFragmentData = NULL;
    Uint32 fragmentLength = 0;
    size_t compressedSize = 0;
    Uint32 fragmentsForThisFrame = 0;
    OFCondition result = EC_Normal;
    OFBool ignoreOffsetTable = cp->ignoreOffsetTable();

    // compute the number of JPEG-LS fragments we need in order to decode the next frame
    fragmentsForThisFrame = computeNumberOfFragments(imageFrames, frameNo, currentItem, ignoreOffsetTable, fromPixSeq);
    if (fragmentsForThisFrame == 0) result = EC_J2KCannotComputeNumberOfFragments;

    // determine planar configuration for uncompressed data
    OFString imageSopClass;
    OFString imagePhotometricInterpretation;
    dataset->findAndGetOFString(DCM_SOPClassUID, imageSopClass);
    dataset->findAndGetOFString(DCM_PhotometricInterpretation, imagePhotometricInterpretation);
    Uint16 imagePlanarConfiguration = 0; // 0 is color-by-pixel, 1 is color-by-plane

    if (imageSamplesPerPixel > 1)
    {
        switch (cp->getPlanarConfiguration())
        {
        case EJ2KPC_restore:
            // get planar configuration from dataset
            imagePlanarConfiguration = 2; // invalid value
            dataset->findAndGetUint16(DCM_PlanarConfiguration, imagePlanarConfiguration);
            // determine auto default if not found or invalid
            if (imagePlanarConfiguration > 1)
                imagePlanarConfiguration = determinePlanarConfiguration(imageSopClass, imagePhotometricInterpretation);
            break;
        case EJ2KPC_auto:
            imagePlanarConfiguration = determinePlanarConfiguration(imageSopClass, imagePhotometricInterpretation);
            break;
        case EJ2KPC_colorByPixel:
            imagePlanarConfiguration = 0;
            break;
        case EJ2KPC_colorByPlane:
            imagePlanarConfiguration = 1;
            break;
        }
    }

    // get the size of all the fragments
    if (result.good())
    {
        // Don't modify the original values for now
        Uint32 fragmentsForThisFrame2 = fragmentsForThisFrame;
        Uint32 currentItem2 = currentItem;

        while (result.good() && fragmentsForThisFrame2--)
        {
            result = fromPixSeq->getItem(pixItem, currentItem2++);
            if (result.good() && pixItem)
            {
                fragmentLength = pixItem->getLength();
                if (result.good()) 
                {
                    if (compressedSize + fragmentLength < compressedSize) 
                    {
                        return EC_MemoryExhausted;
                    }
                    compressedSize += fragmentLength;
                }
            }
        } /* while */
    }

    // get the compressed data
    if (result.good())
    {
        if (compressedSize == 0) 
        {
            return EC_CorruptedData;
        }

        jlsData = new (std::nothrow) Uint8[compressedSize];
        if (!jlsData)
        {
            return EC_MemoryExhausted;
        }

        Uint32 offset = 0;
        while (result.good() && fragmentsForThisFrame--) {
            result = fromPixSeq->getItem(pixItem, currentItem++);
            if (result.good() && pixItem) {
                fragmentLength = pixItem->getLength();
                result = pixItem->getUint8Array(jlsFragmentData);
                if (result.good() && jlsFragmentData) {
                    // prevent overflow into the destination buffer
                    if (offset > compressedSize - fragmentLength) {
                        delete[] jlsData;
                        return EC_CorruptedData;
                    }
                    memcpy(jlsData + offset, jlsFragmentData, fragmentLength);
                    offset += fragmentLength;
                }
            }
        }

        // early cleanup on failure or size mismatch
        if (result.bad()) { delete[] jlsData; return result; }
        if (offset != compressedSize) { delete[] jlsData; return EC_CorruptedData; }
    }

    if (result.good())
    {
        if (compressedSize > 0 && jlsData[compressedSize - 1] == 0) {
            // DICOM encapsulated fragments are padded to even length; a trailing 0x00 is padding.
            --compressedSize;
            if (compressedSize == 0) { delete[] jlsData; return EC_CorruptedData; }
        }

        DecodeData mysrc((unsigned char*)jlsData, compressedSize);

        // start of open jpeg stuff
        opj_dparameters_t parameters;
        opj_codec_t* l_codec = NULL;
        opj_stream_t* l_stream = NULL;
        opj_image_t* image = NULL;

        l_stream = opj_stream_create_memory_stream(&mysrc, OPJ_J2K_STREAM_CHUNK_SIZE, true);
        if (!l_stream) { delete[] jlsData; return EC_MemoryExhausted; }

        // figure out codec
#define JP2_RFC3745_MAGIC "\x00\x00\x00\x0c\x6a\x50\x20\x20\x0d\x0a\x87\x0a"
#define JP2_MAGIC "\x0d\x0a\x87\x0a"
/* position 45: "\xff\x52" */
#define J2K_CODESTREAM_MAGIC "\xff\x4f\xff\x51"

        OPJ_CODEC_FORMAT format = OPJ_CODEC_UNKNOWN;
        if (compressedSize >= 12 && memcmp(jlsData, JP2_RFC3745_MAGIC, 12) == 0)
            format = OPJ_CODEC_JP2;
        else if (compressedSize >= 4 && (memcmp(jlsData, JP2_MAGIC, 4) == 0))
            format = OPJ_CODEC_JP2;
        else if (compressedSize >= 4 && memcmp(jlsData, J2K_CODESTREAM_MAGIC, 4) == 0)
            format = OPJ_CODEC_J2K;
        else
            format = OPJ_CODEC_J2K;

        l_codec = opj_create_decompress(format);
        if (!l_codec) { opj_stream_destroy(l_stream); delete[] jlsData; return EC_MemoryExhausted; }

        opj_set_info_handler(l_codec, msg_callback, NULL);
        opj_set_warning_handler(l_codec, msg_callback, NULL);
        opj_set_error_handler(l_codec, msg_callback, NULL);

        opj_set_default_decoder_parameters(&parameters);

        if (!opj_setup_decoder(l_codec, &parameters))
        {
            opj_stream_destroy(l_stream); l_stream = NULL;
            opj_destroy_codec(l_codec); l_codec = NULL;
            result = EC_CorruptedData;
        }

        if (result.good() && !opj_read_header(l_stream, l_codec, &image))
        {
            opj_stream_destroy(l_stream); l_stream = NULL;
            opj_destroy_codec(l_codec); l_codec = NULL;
            opj_image_destroy(image); image = NULL;
            result = EC_CorruptedData;
        }

        if (result.good())
        {
            if (image->x1 != imageColumns) result = EC_J2KImageDataMismatch;
            else if (image->y1 != imageRows) result = EC_J2KImageDataMismatch;
            else if (image->numcomps != imageSamplesPerPixel) result = EC_J2KImageDataMismatch;
            //else if ((bytesPerSample == 1) && (image->bitspersample > 8)) result = EC_J2KImageDataMismatch;
            //else if ((bytesPerSample == 2) && (image->bitspersample <= 8)) result = EC_J2KImageDataMismatch;
        }

        // validate component precision vs bytesPerSample
        const int prec0 = image->comps[0].prec;  // bits per sample as reported by OpenJPEG
        if ((bytesPerSample == 1 && prec0 > 8) || (bytesPerSample == 2 && prec0 <= 8)) {
            opj_stream_destroy(l_stream); opj_destroy_codec(l_codec); opj_image_destroy(image);
            delete[] jlsData;
            return EC_J2KImageDataMismatch;
        }

        if (!result.good())
        {
            delete[] jlsData;
        }
        else
        {
            if (!(opj_decode(l_codec, l_stream, image) && opj_end_decompress(l_codec, l_stream))) {
                opj_stream_destroy(l_stream); l_stream = NULL;
                opj_destroy_codec(l_codec); l_codec = NULL;
                opj_image_destroy(image); image = NULL;
                result = EC_CorruptedData;
            }

            if (result.good())
            {

                if (image->numcomps == 1) {
                    if (prec0 <= 8) {
                        result = copyUint32ToUint8(image, OFreinterpret_cast(Uint8*, buffer), imageColumns, imageRows);
                    }
                    else {
                        result = copyUint32ToUint16(image, OFreinterpret_cast(Uint16*, buffer), imageColumns, imageRows);
                    }
                }
                else if (image->numcomps == 3) {
                    const int prec0 = image->comps[0].prec;
                    if (image->numcomps == 3 && prec0 > 8) {
                        opj_stream_destroy(l_stream); opj_destroy_codec(l_codec); opj_image_destroy(image);
                        delete[] jlsData;
                        return EC_J2KUnsupportedBitDepth;
                    }
                    if (imagePlanarConfiguration == 0) {
                        result = copyRGBUint8ToRGBUint8(image, OFreinterpret_cast(Uint8*, buffer), imageColumns, imageRows);
                    }
                    else {
                        result = copyRGBUint8ToRGBUint8Planar(image, OFreinterpret_cast(Uint8*, buffer), imageColumns, imageRows);
                    }
                }
                if (result.bad()) {
                    opj_stream_destroy(l_stream); opj_destroy_codec(l_codec); opj_image_destroy(image);
                    delete[] jlsData;
                    return result;
                }
            }

            opj_stream_destroy(l_stream); l_stream = NULL;
            opj_destroy_codec(l_codec); l_codec = NULL;
            opj_image_destroy(image); image = NULL;
            delete[] jlsData;

            if (result.good())
            {
                // decompression is complete, finally adjust byte order if necessary
                if (bytesPerSample == 1) // we're writing bytes into words
                {
                    result = swapIfNecessary(gLocalByteOrder, EBO_LittleEndian, buffer,
                        bufSize, sizeof(Uint16));
                }
            }
        }
    }

    return result;
}


OFCondition DJPEG2KDecoderBase::encode(
    const Uint16* /* pixelData */,
    const Uint32 /* length */,
    const DcmRepresentationParameter* /* toRepParam */,
    DcmPixelSequence*& /* pixSeq */,
    const DcmCodecParameter* /* cp */,
    DcmStack& /* objStack */) const
{
    // we are a decoder only
    return EC_IllegalCall;
}


OFCondition DJPEG2KDecoderBase::encode(
    const Uint16* pixelData,
    const Uint32 length,
    const DcmRepresentationParameter* toRepParam,
    DcmPixelSequence*& pixSeq,
    const DcmCodecParameter* cp,
    DcmStack& objStack,
    OFBool& removeOldRep) const
{
    return EC_IllegalCall;
}


OFCondition DJPEG2KDecoderBase::encode(
    const E_TransferSyntax /* fromRepType */,
    const DcmRepresentationParameter* /* fromRepParam */,
    DcmPixelSequence* /* fromPixSeq */,
    const DcmRepresentationParameter* /* toRepParam */,
    DcmPixelSequence*& /* toPixSeq */,
    const DcmCodecParameter* /* cp */,
    DcmStack& /* objStack */) const
{
    // we don't support re-coding for now.
    return EC_IllegalCall;
}


OFCondition DJPEG2KDecoderBase::encode(
    const E_TransferSyntax fromRepType,
    const DcmRepresentationParameter* fromRepParam,
    DcmPixelSequence* fromPixSeq,
    const DcmRepresentationParameter* toRepParam,
    DcmPixelSequence*& toPixSeq,
    const DcmCodecParameter* cp,
    DcmStack& objStack,
    OFBool& removeOldRep) const
{
    return EC_IllegalCall;
}


OFCondition DJPEG2KDecoderBase::determineDecompressedColorModel(
    const DcmRepresentationParameter* /* fromParam */,
    DcmPixelSequence* /* fromPixSeq */,
    const DcmCodecParameter* /* cp */,
    DcmItem* dataset,
    OFString& decompressedColorModel) const
{
    OFCondition result = EC_IllegalParameter;
    if (dataset != NULL)
    {
        // retrieve color model from given dataset
        result = dataset->findAndGetOFString(DCM_PhotometricInterpretation, decompressedColorModel);
    }
    return result;
}

#if PACKAGE_VERSION_NUMBER > 369
Uint16 DJPEG2KDecoderBase::decodedBitsAllocated(Uint16 bitsAllocated, Uint16 bitsStored) const
{
    // this codec does not support images with less than 2 bits per sample
    if (bitsStored < 2) return 0;

    // for images with 2..8 bits per sample, BitsAllocated will be 8
    if (bitsStored <= 8) return 8;

    // for images with 9..16 bits per sample, BitsAllocated will be 16
    if (bitsStored <= 16) return 16;

    // this codec does not support images with more than 16 bits per sample
    return 0;
}
#endif


Uint16 DJPEG2KDecoderBase::determinePlanarConfiguration(
    const OFString& sopClassUID,
    const OFString& photometricInterpretation)
{
    // Hardcopy Color Image always requires color-by-plane
    if (sopClassUID == UID_RETIRED_HardcopyColorImageStorage) return 1;

    // The 1996 Ultrasound Image IODs require color-by-plane if color model is YBR_FULL.
    if (photometricInterpretation == "YBR_FULL")
    {
        if ((sopClassUID == UID_UltrasoundMultiframeImageStorage)
            || (sopClassUID == UID_UltrasoundImageStorage)) return 1;
    }

    // default for all other cases
    return 0;
}

Uint32 DJPEG2KDecoderBase::computeNumberOfFragments(
    Sint32 numberOfFrames,
    Uint32 currentFrame,
    Uint32 startItem,
    OFBool ignoreOffsetTable,
    DcmPixelSequence* pixSeq)
{

    unsigned long numItems = pixSeq->card();
    DcmPixelItem* pixItem = NULL;

    // We first check the simple cases, that is, a single-frame image,
    // the last frame of a multi-frame image and the standard case where we do have
    // a single fragment per frame.
    if ((numberOfFrames <= 1) || (currentFrame + 1 == OFstatic_cast(Uint32, numberOfFrames)))
    {
        // single-frame image or last frame. All remaining fragments belong to this frame
        return (numItems - startItem);
    }
    if (OFstatic_cast(Uint32, numberOfFrames + 1) == numItems)
    {
        // multi-frame image with one fragment per frame
        return 1;
    }

    OFCondition result = EC_Normal;
    if (!ignoreOffsetTable)
    {
        // We do have a multi-frame image with multiple fragments per frame, and we are
        // not working on the last frame. Let's check the offset table if present.
        result = pixSeq->getItem(pixItem, 0);
        if (result.good() && pixItem)
        {
            Uint32 offsetTableLength = pixItem->getLength();
            if (offsetTableLength == (OFstatic_cast(Uint32, numberOfFrames) * 4))
            {
                // offset table is non-empty and contains one entry per frame
                Uint8* offsetData = NULL;
                result = pixItem->getUint8Array(offsetData);
                if (result.good() && offsetData)
                {
                    // now we can access the offset table
                    Uint32* offsetData32 = OFreinterpret_cast(Uint32*, offsetData);

                    // extract the offset for the NEXT frame. This offset is guaranteed to exist
                    // because the "last frame/single frame" case is handled above.
                    Uint32 offset = offsetData32[currentFrame + 1];

                    // convert to local endian byte order (always little endian in file)
                    swapIfNecessary(gLocalByteOrder, EBO_LittleEndian, &offset, sizeof(Uint32), sizeof(Uint32));

                    // determine index of start fragment for next frame
                    Uint32 byteCount = 0;
                    Uint32 fragmentIndex = 1;
                    while ((byteCount < offset) && (fragmentIndex < numItems))
                    {
                        pixItem = NULL;
                        result = pixSeq->getItem(pixItem, fragmentIndex++);
                        if (result.good() && pixItem)
                        {
                            byteCount += pixItem->getLength() + 8; // add 8 bytes for item tag and length
                            if ((byteCount == offset) && (fragmentIndex > startItem))
                            {
                                // bingo, we have found the offset for the next frame
                                return fragmentIndex - startItem;
                            }
                        }
                        else break; /* something went wrong, break out of while loop */
                    } /* while */
                }
            }
        }
    }

    // So we have a multi-frame image with multiple fragments per frame and the
    // offset table is empty or wrong. Our last chance is to peek into the JPEG-2000
    // bistream and identify the start of the next frame.
    Uint32 nextItem = startItem;
    Uint8* fragmentData = NULL;
    while (++nextItem < numItems)
    {
        pixItem = NULL;
        result = pixSeq->getItem(pixItem, nextItem);
        if (result.good() && pixItem)
        {
            fragmentData = NULL;
            result = pixItem->getUint8Array(fragmentData);
            if (result.good() && fragmentData && (pixItem->getLength() > 3))
            {
                if (isJPEGLSStartOfImage(fragmentData))
                {
                    // found a JPEG-2000 SOI marker. Assume that this is the start of the next frame.
                    return (nextItem - startItem);
                }
            }
            else break; /* something went wrong, break out of while loop */
        }
        else break; /* something went wrong, break out of while loop */
    }

    // We're bust. No way to determine the number of fragments per frame.
    return 0;
}

OFBool DJPEG2KDecoderBase::isJPEGLSStartOfImage(Uint8* fragmentData)
{
    // A valid JPEG-2000 bitstream will always start with an SOI marker FFD8, followed
    // by either an SOF55 (FFF7), COM (FFFE) or APPn (FFE0-FFEF) marker.
    if ((*fragmentData++) != 0xFF) return OFFalse;
    if ((*fragmentData++) != 0xD8) return OFFalse;
    if ((*fragmentData++) != 0xFF) return OFFalse;
    if ((*fragmentData == 0xF7) || (*fragmentData == 0xFE) || ((*fragmentData & 0xF0) == 0xE0))
    {
        return OFTrue;
    }
    return OFFalse;
}


OFCondition DJPEG2KDecoderBase::createPlanarConfiguration1Byte(
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint8* buf = new Uint8[3 * numPixels + 3];
    if (buf)
    {
        memcpy(buf, imageFrame, (size_t)(3 * numPixels));
        Uint8* s = buf;                        // source
        Uint8* r = imageFrame;                 // red plane
        Uint8* g = imageFrame + numPixels;     // green plane
        Uint8* b = imageFrame + (2 * numPixels); // blue plane
        for (unsigned long i = numPixels; i; i--)
        {
            *r++ = *s++;
            *g++ = *s++;
            *b++ = *s++;
        }
        delete[] buf;
    }
    else return EC_MemoryExhausted;
    return EC_Normal;
}


OFCondition DJPEG2KDecoderBase::createPlanarConfiguration1Word(
    Uint16* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint16* buf = new Uint16[3 * numPixels + 3];
    if (buf)
    {
        memcpy(buf, imageFrame, (size_t)(3 * numPixels * sizeof(Uint16)));
        Uint16* s = buf;                        // source
        Uint16* r = imageFrame;                 // red plane
        Uint16* g = imageFrame + numPixels;     // green plane
        Uint16* b = imageFrame + (2 * numPixels); // blue plane
        for (unsigned long i = numPixels; i; i--)
        {
            *r++ = *s++;
            *g++ = *s++;
            *b++ = *s++;
        }
        delete[] buf;
    }
    else return EC_MemoryExhausted;
    return EC_Normal;
}

OFCondition DJPEG2KDecoderBase::createPlanarConfiguration0Byte(
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint8* buf = new Uint8[3 * numPixels + 3];
    if (buf)
    {
        memcpy(buf, imageFrame, (size_t)(3 * numPixels));
        Uint8* t = imageFrame;          // target
        Uint8* r = buf;                 // red plane
        Uint8* g = buf + numPixels;     // green plane
        Uint8* b = buf + (2 * numPixels); // blue plane
        for (unsigned long i = numPixels; i; i--)
        {
            *t++ = *r++;
            *t++ = *g++;
            *t++ = *b++;
        }
        delete[] buf;
    }
    else return EC_MemoryExhausted;
    return EC_Normal;
}


OFCondition DJPEG2KDecoderBase::createPlanarConfiguration0Word(
    Uint16* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint16* buf = new Uint16[3 * numPixels + 3];
    if (buf)
    {
        memcpy(buf, imageFrame, (size_t)(3 * numPixels * sizeof(Uint16)));
        Uint16* t = imageFrame;          // target
        Uint16* r = buf;                 // red plane
        Uint16* g = buf + numPixels;     // green plane
        Uint16* b = buf + (2 * numPixels); // blue plane
        for (unsigned long i = numPixels; i; i--)
        {
            *t++ = *r++;
            *t++ = *g++;
            *t++ = *b++;
        }
        delete[] buf;
    }
    else return EC_MemoryExhausted;
    return EC_Normal;
}


OFCondition copyUint32ToUint8(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint8* t = imageFrame;          // target
    OPJ_INT32* g = image->comps[0].data;   // grey plane  
    for (unsigned long i = numPixels; i; i--)
    {
        *t++ = *g++;
    }

    return EC_Normal;
}

OFCondition copyUint32ToUint16(
    opj_image_t* image,
    Uint16* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint16* t = imageFrame;          // target
    OPJ_INT32* g = image->comps[0].data;   // grey plane  
    for (unsigned long i = numPixels; i; i--)
    {
        *t++ = *g++;
    }

    return EC_Normal;
}

OFCondition copyRGBUint8ToRGBUint8(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint8* t = imageFrame;          // target
    OPJ_INT32* r = image->comps[0].data; // red plane
    OPJ_INT32* g = image->comps[1].data; // green plane
    OPJ_INT32* b = image->comps[2].data; // blue plane
    for (unsigned long i = numPixels; i; i--)
    {
        *t++ = *r++;
        *t++ = *g++;
        *t++ = *b++;
    }

    return EC_Normal;
}

OFCondition copyRGBUint8ToRGBUint8Planar(
    opj_image_t* image,
    Uint8* imageFrame,
    Uint16 columns,
    Uint16 rows)
{
    if (imageFrame == NULL) return EC_IllegalCall;

    unsigned long numPixels = columns * rows;
    if (numPixels == 0) return EC_IllegalCall;

    Uint8* t = imageFrame;          // target
    for (unsigned long j = 0; j < 3; j++)
    {
        OPJ_INT32* r = image->comps[j].data; // color plane  
        for (unsigned long i = numPixels; i; i--)
        {
            *t++ = *r++;
        }
    }
    return EC_Normal;
}
