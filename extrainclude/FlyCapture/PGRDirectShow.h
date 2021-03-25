//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: PGRDirectShow.h,v 1.17 2011-02-11 17:08:54 warrenm Exp $
//=============================================================================

//=============================================================================
//
// PGRDirectShow.h
//
//   Defines the API to the PGR DirectShow library.
//
//  We welcome your bug reports, suggestions, and comments:
//  www.ptgrey.com/support/contact
//
//=============================================================================

#ifndef _PGRDIRECTSHOW_H_
#define _PGRDIRECTSHOW_H_

#include "SpinFlyIncludes.h"
#include <vector>

const unsigned int MAX_LENGTH = 256;

namespace
	{
		// TODO: this cannot be in the public header
#ifdef LINK_TO_SPIN
		const unsigned int FULL_32BIT_VALUE = 0x7FFFFFFF;
#endif
		// This should be identical to the SpinFlyAdapter pixel format enum
		enum PixelFormat
		{        
			PIXEL_FORMAT_MONO8,
			PIXEL_FORMAT_411YUV8,
			PIXEL_FORMAT_422YUV8,
			PIXEL_FORMAT_444YUV8,
			PIXEL_FORMAT_RGB8,
			PIXEL_FORMAT_MONO16,
			PIXEL_FORMAT_RGB16,
			PIXEL_FORMAT_S_MONO16,
			PIXEL_FORMAT_S_RGB16,
			PIXEL_FORMAT_RAW8,
			PIXEL_FORMAT_RAW16,
			PIXEL_FORMAT_MONO12,
			PIXEL_FORMAT_RAW12,
			PIXEL_FORMAT_BGR,
			PIXEL_FORMAT_BGRU,
			PIXEL_FORMAT_RGB,
			PIXEL_FORMAT_RGBU,
			PIXEL_FORMAT_BGR16,
			PIXEL_FORMAT_BGRU16,
			PIXEL_FORMAT_422YUV8_JPEG,
			NUM_PIXEL_FORMATS	   =  20, /**< Number of pixel formats. */
			UNSPECIFIED_PIXEL_FORMAT = 0 /**< Unspecified pixel format. */
		};

		enum ColorProcessingAlgorithm
		{        
			DEFAULT, 
			NO_COLOR_PROCESSING, 
			NEAREST_NEIGHBOR, 
			EDGE_SENSING, 
			HQ_LINEAR,
			RIGOROUS,
			IPP,
			DIRECTIONAL_FILTER,
			COLOR_PROCESSING_ALGORITHM_FORCE_32BITS = FULL_32BIT_VALUE
		};
	}

/** DCAM formats supported by the camera */
struct OutputFormatElement
{
	float framerate;
	unsigned int width;
	int height;
	unsigned int bitCount;
	DWORD compression;
	GUID mediaSubType;
	REFERENCE_TIME avgFrameTime;

	unsigned int f7Mode;
	unsigned int xOffset;
	unsigned int yOffset;
	unsigned int packetSize;
	PixelFormat pixelFormat;

	std::vector<float> frameRates;
};

struct TriggerMode
{      
	bool onOff;
	unsigned int polarity;
	unsigned int source;
	unsigned int mode;
	unsigned int parameter;      
	unsigned int reserved[8];

	TriggerMode()
	{
		onOff = false;
		polarity = 0;
		source = 0;
		mode = 0;
		parameter = 0;
		memset( reserved, 0, sizeof(reserved) );
	}
};

/** Supported output modes for source filter */
const enum SupportedOutputColorModes 
{
	RGB32,
	RGB24,
	YUY2,
	UYVY,
	RAW
};

/** A camera list entry */
struct CameraListEntry
{
	unsigned int serialNumber;
	wchar_t model[MAX_LENGTH];
};

/** 
 * This is the Interface that allows Users to get and set Properties on 
 * the camera. You can query the PGRDirectShowSource Capture Filter for 
 * IID_IFlyCaptureProperties and it will return a pointer to the 
 * IFlyCaptureProperties interface.
 * The GUID is equivalent to {2BD99656-1552-4d98-B648-0DD0196D1649}.
 */
const IID IID_IFlyCaptureProperties = {0x2bd99656, 0x1552, 0x4d98, {0xb6,0x48,0xd,0xd0,0x19,0x6d,0x16,0x49}};

interface IFlyCaptureProperties : public IUnknown
{
    /** 
     * @name DCAM Formats
     *
     * These functions deal with DCAM format selection.
     */
    /*@{*/ 

    /**
     * Get formats supported by the camera.
     *
     * @param pFormats DCAM formats supported by the camera.
     *
     * @return An HRESULT indicating the success or failure of the function.
     */
	STDMETHOD(GetAvailableFormats)(std::vector<struct OutputFormatElement> *pFormats) = 0;

	STDMETHOD(GetOutputColorMode)(SupportedOutputColorModes* outputColorMode) = 0;
	STDMETHOD(SetOutputColorMode)(SupportedOutputColorModes outputColorMode) = 0;
	STDMETHOD(GetColorAlgorithm)(ColorProcessingAlgorithm* colorAlgorithm) = 0;
	STDMETHOD(SetColorAlgorithm)(ColorProcessingAlgorithm colorAlgorithm) = 0;

	/**
     * Returns the index of the format currently set. The index refers to 
     * the index of the videoModes array in the DCAMFormats structure returned
     * by a call to GetAvailableFormats().
	 *
	 * @param pulFormat Index of currently set format.
	 *
	 * @return An HRESULT indicating the success or failure of the function.
	 */
	STDMETHOD(GetImageFormat)(unsigned long *pulFormat) = 0;

    /**
     * Sets the index of the format to be used. The index refers to 
     * the index of the videoModes array in the DCAMFormats structure returned
     * by a call to GetAvailableFormats().
     *
     * @param ulFormat index of the format to be set.
     *
     * @return An HRESULT error code indicating the success or failure of the function.
     */
    STDMETHOD(SetImageFormat)(unsigned long ulFormat) = 0;	
	STDMETHOD(SetImageFormat)(unsigned long ulFormat, SupportedOutputColorModes outputColorMode) = 0;	

	/*@}*/ 

    /** 
     * @name Register functions
     *
     * These functions deal with register manipulation on the camera.
     */
    /*@{*/ 

	/**
	 * Read the specified register from the camera.
	 *
	 * @param offset DCAM address to be read from.
	 * @param pValue The value that is read.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(ReadRegister)(unsigned int offset, unsigned int* pValue) = 0;

    /**
     * Write to the specified register on the camera.
     *
     * @param offset DCAM address to be written to.
     * @param value The value to be written.
     * @param broadcast Whether the action should be broadcast.
     *
     * @return An HRESULT error code indicating the success or failure of the function.
     */
    STDMETHOD(WriteRegister)(unsigned int offset, unsigned int value, bool broadcast = false) = 0;

    /*@}*/ 

    /** 
     * @name Camera settings functions
     *
     * These functions deal with camera settings that affect the image,
     * such as gain, shutter speed etc. There are six functions for each
     * property. There are three functions each for relative and absolute
     * values.
     */
    /*@{*/ 

    /*@}*/ 

    /** 
     * @name Brightness functions
     *
     * These functions deal with brightness control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsBrightnessSupported)() = 0;
	STDMETHOD(GetBrightness)(long* pBrightness, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetBrightness)(long brightness, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetBrightnessRange)(long* pMin,long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsBrightness)(float* pBrightness, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsBrightness)(float brightness, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsBrightnessRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

	/*@}*/ 

    /** 
     * @name Exposure functions
     *
     * These functions deal with exposure control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsExposureSupported)() = 0;
	STDMETHOD(GetExposure)(long* pExposure, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetExposure)(long exposure, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetExposureRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsExposure)(float* pExposure, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsExposure)(float exposure, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsExposureRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Shutter functions
     *
     * These functions deal with shutter control on the camera.
     */
    /*@{*/ 
	
	STDMETHOD_(bool, IsShutterSupported)() = 0;
	STDMETHOD(GetShutter)(long* pShutter, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetShutter)(long shutter, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetShutterRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsShutter)(float* pShutter, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsShutter)(float shutter, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsShutterRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Sharpness functions
     *
     * These functions deal with sharpness control on the camera.
     */
    /*@{*/ 
		
	STDMETHOD_(bool, IsSharpnessSupported)() = 0;
	STDMETHOD(GetSharpness)(long* pSharpness, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetSharpness)(long sharpness, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetSharpnessRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsSharpness)(float* plSharpness, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsSharpness)(float lSharpness, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsSharpnessRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Gain functions
     *
     * These functions deal with gain control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsGainSupported)() = 0;
	STDMETHOD(GetGain)(long* pGain, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetGain)(long gain, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetGainRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsGain)(float* gain, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsGain)(float lGain, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsGainRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

	/*@}*/ 

    /** 
     * @name Hue functions
     *
     * These functions deal with hue control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsHueSupported)() = 0;
	STDMETHOD(GetHue)(long* pHue, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetHue)(long hue, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetHueRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsHue)(float* pHue, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsHue)(float hue, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsHueRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Saturation functions
     *
     * These functions deal with saturation control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsSaturationSupported)() = 0;
	STDMETHOD(GetSaturation)(long* pSaturation,bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetSaturation)(long saturation, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetSaturationRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsSaturation)(float* pSaturation,bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsSaturation)(float saturation, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsSaturationRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Gamma functions
     *
     * These functions deal with gamma control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsGammaSupported)() = 0;
	STDMETHOD(GetGamma)(long* pGamma, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetGamma)(long gamma, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetGammaRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsGamma)(float* pGamma, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsGamma)(float gamma, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsGammaRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Pan functions
     *
     * These functions deal with pan control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsPanSupported)() = 0;
	STDMETHOD(GetPan)(long* pPan, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetPan)(long pan, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetPanRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsPan)(float* pPan, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsPan)(float pan, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsPanRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name Tilt functions
     *
     * These functions deal with tilt control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsTiltSupported)() = 0;
	STDMETHOD(GetTilt)(long* pTilt, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetTilt)(long tilt, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetTiltRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsTilt)(float* pTilt, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsTilt)(float tilt, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsTiltRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

	/*@}*/ 

    /** 
     * @name FrameRate functions
     *
     * These functions deal with frame rate control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsFrameRateSupported)() = 0;
	STDMETHOD(GetFrameRate)(long* pFrameRate, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetFrameRate)(long FrameRate, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetFrameRateRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsFrameRate)(float* pFrameRate, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsFrameRate)(float FrameRate, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsFrameRateRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /** 
     * @name White balance functions
     *
     * These functions deal with white balance control on the camera.
     */
    /*@{*/ 

	STDMETHOD_(bool, IsWhiteBalanceSupported)() = 0;
	STDMETHOD(GetWhiteBalance)(long* plWhiteBalance, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetWhiteBalance)(long lWhiteBalance, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetWhiteBalanceRange)(long* pMin, long* pMax, bool* pIsAutoSupported = NULL) = 0;
	STDMETHOD(GetAbsWhiteBalance)(float* plWhiteBalance, bool* pIsAutoEnabled = NULL) = 0;
	STDMETHOD(SetAbsWhiteBalance)(float lWhiteBalance, bool isAutoEnabled = false) = 0;
	STDMETHOD(GetAbsWhiteBalanceRange)(float* pMin, float* pMax, bool* pIsAutoSupported = NULL, const char** pUnits = NULL) = 0;

    /*@}*/ 

    /*@}*/ 

    /** 
     * @name Custom Image / Format7 functions
     *
     * These functions deal with custom image and Format7 image control.
     */
    /*@{*/ 

	/**
	 * Get whether the image is vertically flipped.
	 *
	 * @param pIsVerticallyFlipped Whether the image is flipped.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(GetOutputVerticalFlip)(bool* pIsVerticallyFlipped) = 0;

	/**
	 * Set the image to be vertically flipped (or not).
	 *
	 * @param enableVerticalFlip Whether the image should be vertically flipped.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(SetOutputVerticalFlip)(bool enableVerticalFlip) = 0;

	/**
	 * Get whether the camera is in custom image / Format7 mode.
	 *
	 * @param pIsInCustomImageMode Whether the camera is in custom image / Format7 mode.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(GetCustomImageMode)(bool* pIsInCustomImageMode) = 0;

	/**
	 * Get information for a particular custom image / Format7 mode.
	 *
	 * @param mode Mode to query.
	 * @param pAvailable Whether mode is supported.
	 * @param pMaxWidth Maximum width.
	 * @param pMaxHeight Maximum height.
	 * @param pPixelFormatsBitMask Pixel format bit mask.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(QueryCustomImage)(
        unsigned int mode,
        //bool* pAvailable,
        unsigned int* pMaxWidth,
        unsigned int* pMaxHeight, 
		std::vector<PixelFormat>* pPixFormats
        /*unsigned int* pPixelFormatsBitMask */) = 0;

    /**
	 * Get information for a particular custom image / Format7 mode.
	 *
	 * @param mode Mode to query.
	 * @param pAvailable Whether mode is supported.
	 * @param pMaxWidth Maximum width.
	 * @param pMaxHeight Maximum height.
	 * @param pPixFormats Pixel format bit mask.
	 * @param pOffsetHStepSize Horizontal step size for the offset. 
	 * @param pOffsetVStepSize Vertical step size for the offset. 
	 * @param pImageHStepSize Horizontal step size for the image. 
	 * @param pImageVStepSize Vertical step size for the image. 
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(QueryCustomImage)(
		unsigned int mode,
		//bool* pAvailable,
		unsigned int* pMaxWidth,
		unsigned int* pMaxHeight, 
		//unsigned int* pPixFormats,
		std::vector<PixelFormat>* pPixFormats,
		unsigned int* pOffsetHStepSize,
		unsigned int* pOffsetVStepSize,
		unsigned int* pImageHStepSize,
		unsigned int* pImageVStepSize ) = 0;

	/**
	 * Set custom image / Format7 image settings to camera.
	 *
	 * @param mode Custom image / Format7 settings to use.
	 * @param left Left offset.
	 * @param top Top offset.
	 * @param width Image width.
	 * @param height Image height.
	 * @param pixelFormat Pixel format index to use.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(SetCustomImage)(
        unsigned int mode,
        unsigned int left,
        unsigned int top,
        unsigned int width,
        unsigned int height, 
		PixelFormat pixFormat
        /*unsigned int pixelFormat*/ ) = 0;

	/**
	 * Get custom image / Format7 settings from camera.
	 *
	 * @param pMode Custom image / Format7 settings on camera.
	 * @param pLeft Left offset.
	 * @param pTop Top offset.
	 * @param pWidth Image width.
	 * @param pHeight Image height.
	 * @param pPixelFormat Pixel format.
	 *
	 * @return An HRESULT error code indicating the success or failure of the function.
	 */
	STDMETHOD(GetCustomImage)(
        unsigned int* pMode,
        unsigned int* pLeft,
        unsigned int* pTop,
        unsigned int* pWidth,
        unsigned int* pHeight, 
        unsigned int* pPixelFormat ) = 0;	

	STDMETHOD(EnumerateCameras)(std::vector<CameraListEntry>* camList) = 0;
	STDMETHOD(ConnectToCamera)(unsigned int serialNum) = 0;
};

#endif // _PGRDIRECTSHOW_H_