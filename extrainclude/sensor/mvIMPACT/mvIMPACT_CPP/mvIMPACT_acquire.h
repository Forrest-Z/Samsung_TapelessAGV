//-----------------------------------------------------------------------------
#ifndef MVIMPACT_ACQUIRE_H_
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#   define MVIMPACT_ACQUIRE_H_ MVIMPACT_ACQUIRE_H_
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
//-----------------------------------------------------------------------------
#ifdef SWIG
#   ifdef SWIGPYTHON
#       define WRAP_PYTHON
#   endif
#   ifdef SWIGJAVA
#       define WRAP_JAVA
#   endif
#endif

#ifdef WRAP_PYTHON
#   define PYTHON_ONLY(X) X
#   define PYTHON_CPP_SWITCH(PYTHON_WRAPPER_CODE,CPP_WRAPPER_CODE) PYTHON_WRAPPER_CODE
#   ifndef WRAP_ANY
#       define WRAP_ANY
#   endif // #ifndef WRAP_ANY
#else // #ifdef WRAP_PYTHON
#   define PYTHON_ONLY(X)
#   define PYTHON_CPP_SWITCH(PYTHON_WRAPPER_CODE,CPP_WRAPPER_CODE) CPP_WRAPPER_CODE
#endif // #ifdef WRAP_PYTHON

#ifdef WRAP_DOTNET
#   define DOTNET_ONLY(X) X
#   define DOTNET_CPP_SWITCH(DOTNET_WRAPPER_CODE,CPP_WRAPPER_CODE) DOTNET_WRAPPER_CODE
#   ifndef WRAP_ANY
#       define WRAP_ANY
#   endif // #ifndef WRAP_ANY
#else // #ifdef WRAP_DOTNET
#   define DOTNET_ONLY(X)
#   define DOTNET_CPP_SWITCH(DOTNET_WRAPPER_CODE,CPP_WRAPPER_CODE) CPP_WRAPPER_CODE
#endif // #ifdef WRAP_DOTNET

#if !defined(MVIMPACT_DEPRECATED_CPP)
#   ifdef WRAP_PYTHON
#       define MVIMPACT_DEPRECATED_CPP(FUNCTION)
#   elif defined(__GNUC__) && (__GNUC__ >= 3) && defined(__GNUC_MINOR__) && (__GNUC_MINOR__ >= 1) // is at least GCC 3.1 compiler?
#       define MVIMPACT_DEPRECATED_CPP(FUNCTION) FUNCTION __attribute__ ((deprecated))
#   elif defined(_MSC_VER) && (_MSC_VER >= 1300) // is at least VC 2003 compiler?
#       define MVIMPACT_DEPRECATED_CPP(FUNCTION) __declspec(deprecated) FUNCTION
#   else
#       define MVIMPACT_DEPRECATED_CPP(FUNCTION) FUNCTION
#   endif // compiler check
#endif // #if !defined(MVIMPACT_DEPRECATED_CPP) && !defined(DOXYGEN_SHOULD_SKIP_THIS)

#if !defined(DOXYGEN_SHOULD_SKIP_THIS)
#   ifdef _MSC_VER // is Microsoft compiler?
#       pragma warning( push )
#       if _MSC_VER < 1300 // is 'old' VC 6 compiler?
#           pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#           define MVIA_FUNCTION "No function name information as the __FUNCTION__ macro is not supported by this(VC 6) compiler"
#           pragma message( "WARNING: This header(" __FILE__ ") uses the __FUNCTION__ macro, which is not supported by this compiler. A default definition(\"" MVIA_FUNCTION "\") will be used!" )
#           pragma message( "WARNING: This header(" __FILE__ ") uses inheritance for exception classes. However this compiler can't handle this correctly. Trying to catch a specific exception by writing a catch block for a base class will not work!" )
#       else
#           define MVIA_FUNCTION __FUNCTION__
#       endif // #if _MSC_VER < 1300
#       pragma warning( disable : 4512 ) // 'assignment operator could not be generated' (reason: assignment operators declared 'private' but not implemented)
#   elif defined(__BORLANDC__) // is Borland compiler?
#       pragma option push -b // force enums to the size of integer
#       define MVIA_FUNCTION __FUNC__
#   else
#       define MVIA_FUNCTION __FUNCTION__
#   endif // #ifdef _MSC_VER
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY

#ifndef WRAP_ANY
#   ifdef _MSC_VER // is Microsoft compiler?
#       if _MSC_VER >= 1400 // is at least VC 2005 compiler?
#           include <assert.h>
#       endif // #if _MSC_VER >= 1400
#   endif // #ifdef _MSC_VER
#   include <limits.h>
#   include <map>
#   ifdef __BORLANDC__ // is Borland compiler?
#       include <mem.h> // Borland has own ideas about where 'memset' should be defined...
#   endif // #ifdef __BORLANDC__
#   include <set>
#   include <sstream>
#   include <stdexcept>
#   include <stdlib.h>
#   include <string>
#   include <string.h>
#   include <vector>
#endif // #ifdef WRAP_ANY

#ifndef WRAP_ANY
#   ifdef MVIMPACT_H_
namespace mvIMPACT
{
// needed to keep the C-header working
using mvIMPACT_C::IPL_BUFHANDLE;
} // namespace mvIMPACT
#   endif // #ifdef MVIMPACT_H_
#endif // #ifndef WRAP_ANY

// C-API header will be imported into mvIMPACT::acquire if included from here
#include <mvDeviceManager/Include/mvDeviceManager.h>

namespace mvIMPACT
{
/// \namespace mvIMPACT::acquire This namespace contains classes and functions belonging to the image acquisition module of this SDK.
namespace acquire
{

/// \defgroup CommonInterface Common
/// \brief Classes and functions that are available for all interface layouts.
///
/// This group contains classes and functions that are available for all interface layouts.
///
/// @{

// predeclares
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class CameraDescriptionManager;
class CameraSettingsBlueCOUGAR;
class CameraSettingsBlueFOX;
class ComponentList;
class ComponentLocator;
class Device;
class DigitalInput;
class DigitalOutput;
class EventSubSystem;
class HDRControl;
class IOSubSystem;
class ImageProcessing;
class ImageRequestControl;
class Request;
class RTCtrProgram;
class RTCtrProgramStep;
class WhiteBalanceSettings;
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef WRAP_DOTNET
#   ifdef MVIMPACT_H_
//-----------------------------------------------------------------------------
/// \brief An base class for exceptions generated by mvIMPACT Acquire.
/**
 * \if DOXYGEN_CPP_DOCUMENTATION
 * If \a mvIMPACT.h has been included before including this interface, this class will
 * be derived from \b mvIMPACT::ImpactException, which is derived from \b std::runtime_error.
 * If this interface is used without the rest of the \b mvIMPACT library, this class
 * will be derived directly from \b std::runtime_error, but apart from that will provide the
 * same interface and class layout.
 * \endif
*/
class ImpactAcquireException : public mvIMPACT::ImpactException
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ImpactAcquireException object.
    explicit ImpactAcquireException(
        /// [in] The string representation of the error.
        const std::string& errorString,
        /// [in] The origin of the error (function name and line)
        const std::string& errorOrigin,
        /// [in] The numerical representation of the error.
        int errorCode ) : ImpactException( errorString, errorOrigin, errorCode ) {}
    virtual ~ImpactAcquireException() throw() {}
    /// \brief Returns a string representation of the error associated with the exception.
    /**
     * This function will return the name of the enum of the error code.
     *
     * \b EXAMPLE
     *
     * In case of error \b -2102 this function would return \b mvIMPACT::acquire::DMR_DRV_ALREADY_IN_USE, which
     * is the error codes corresponding enum name.
     */
    std::string getErrorCodeAsString( void ) const
    {
        return DMR_ErrorCodeToString( getErrorCode() );
    }
    /// \brief Returns a string representation of a error.
    /**
     * This function will return the name of the enum of the error code.
     *
     * \b EXAMPLE
     *
     * In case of error \b -2102 this function would return \b mvIMPACT::acquire::DMR_DRV_ALREADY_IN_USE, which
     * is the error codes corresponding enum name.
     */
    static std::string getErrorCodeAsString(
        /// [in] The error code to query a string representation for.
        int errorCode )
    {
        return DMR_ErrorCodeToString( errorCode );
    }
};
#   else
//-----------------------------------------------------------------------------
/// \brief An base class for exceptions generated by mvIMPACT Acquire.
class ImpactAcquireException : public std::runtime_error
//-----------------------------------------------------------------------------
{
    const int m_errorCode;
    const std::string m_errorOrigin;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ImpactAcquireException object.
    explicit ImpactAcquireException(
        /// [in] The string representation of the error.
        const std::string& errorString,
        /// [in] The origin of the error (function name and line)
        const std::string& errorOrigin,
        /// [in] The numerical representation of the error.
        int errorCode ) : std::runtime_error( errorString ), m_errorCode( errorCode ), m_errorOrigin( errorOrigin ) {}
    virtual ~ImpactAcquireException() throw() {}
    /// \brief Returns an error string containing information about the reason for the error.
    std::string getErrorString( void ) const
    {
        return what();
    }
    /// \brief Returns a unique numerical representation for this error.
    int getErrorCode( void ) const
    {
        return m_errorCode;
    }
    /// \brief Returns a string representation of the error associated with the exception.
    /**
     * This function will return the name of the enum of the error code.
     *
     * \b EXAMPLE
     *
     * In case of error \b -2102 this function would return \b mvIMPACT::acquire::DMR_DRV_ALREADY_IN_USE, which
     * is the error codes corresponding enum name.
     */
    std::string getErrorCodeAsString( void ) const
    {
        return DMR_ErrorCodeToString( m_errorCode );
    }
    /// \brief Returns a string representation of a error.
    /**
     * This function will return the name of the enum of the error code.
     *
     * \b EXAMPLE
     *
     * In case of error \b -2102 this function would return \b mvIMPACT::acquire::DMR_DRV_ALREADY_IN_USE, which
     * is the error codes corresponding enum name.
     */
    static std::string getErrorCodeAsString(
        /// [in] The error code to query a string representation for.
        int errorCode )
    {
        return DMR_ErrorCodeToString( errorCode );
    }
    /// \brief Returns information about the origin of the error.
    /**
     * The string returned by this function will contain the name of the function
     * and the line number where the exception was raised.
     */
    std::string getErrorOrigin( void ) const
    {
        return m_errorOrigin;
    }
};
#   endif // #ifdef MVIMPACT_H_

//-----------------------------------------------------------------------------
/// \brief A base class for device manager related exceptions.
class EDeviceManager : public ImpactAcquireException
//-----------------------------------------------------------------------------
{
public:
    /// \brief Creates a new \b mvIMPACT::acquire::EDeviceManager object.
    explicit EDeviceManager(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TDMR_ERROR errorCode ) : ImpactAcquireException( errorString, errorOrigin, errorCode ) {}
    virtual ~EDeviceManager() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief A base class for exceptions related to the property module.
/**
 * This exception object will only be raised if an unknown error occurred.
 * To check for errors related to the property module a catch block for
 * \b mvIMPACT::acquire::EPropertyHandling objects can be written to catch all property module related
 * exceptions.
 */
class EPropertyHandling : public ImpactAcquireException
//-----------------------------------------------------------------------------
{
public:
    /// \brief Creates a new \b mvIMPACT::acquire::EPropertyHandling object.
    explicit EPropertyHandling(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TPROPHANDLING_ERROR errorCode ) : ImpactAcquireException( errorString, errorOrigin, errorCode ) {}
    virtual ~EPropertyHandling() throw() {}
};

//-----------------------------------------------------------------------------
// COMPONENT EXCEPTION CLASSES :
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// \brief A base class for \b mvIMPACT::acquire::Component object related exceptions from the property module.
/**
 * This exception object will never be raised in the code. It can't be constructed
 * directly but to check for \b mvIMPACT::acquire::Component related errors a catch block for
 * \b mvIMPACT::acquire::EComponent objects can be written to catch all \b mvIMPACT::acquire::Component related
 * exceptions.
 */
class EComponent : public EPropertyHandling
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Creates a new \b mvIMPACT::acquire::EComponent object.
    explicit EComponent(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TPROPHANDLING_ERROR errorCode ) : EPropertyHandling( errorString, errorOrigin, errorCode ) {}
    virtual ~EComponent() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NOT_A_LIST error.
class ENotAList : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENotAList( const std::string& componentname, const std::string& errorOrigin ) : EComponent( "Component " + componentname + " is not a list. Unable to perform list operation", errorOrigin, PROPHANDLING_NOT_A_LIST ) {}
    virtual ~ENotAList() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NOT_A_PROPERTY error.
class ENotAProperty : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENotAProperty( const std::string& componentname, const std::string& errorOrigin ) : EComponent( "Component " + componentname + " is not a property. Unable to perform property operation", errorOrigin, PROPHANDLING_NOT_A_PROPERTY ) {}
    virtual ~ENotAProperty() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NOT_A_METHOD error.
class ENotAMethod : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENotAMethod( const std::string& componentname, const std::string& errorOrigin ) : EComponent( "Component " + componentname + " is not a method. Unable to perform function call operation", errorOrigin, PROPHANDLING_NOT_A_METHOD ) {}
    virtual ~ENotAMethod() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NO_READ_RIGHTS error.
class ENoReadRights : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENoReadRights( const std::string& compname, const std::string& errorOrigin ) : EComponent( "No read rights for component " + compname, errorOrigin, PROPHANDLING_NO_READ_RIGHTS ) {}
    virtual ~ENoReadRights() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NO_WRITE_RIGHTS error.
class ENoWriteRights : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENoWriteRights( const std::string& compname, const std::string& errorOrigin ) : EComponent( "No write rights for component " + compname, errorOrigin, PROPHANDLING_NO_WRITE_RIGHTS ) {}
    virtual ~ENoWriteRights() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NO_MODIFY_SIZE_RIGHTS error.
class ENoModifySizeRights : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENoModifySizeRights( const std::string& compname, const std::string& errorOrigin ) : EComponent( "Component " + compname + "s size is fixed and therefore cannot be modified", errorOrigin, PROPHANDLING_NO_MODIFY_SIZE_RIGHTS ) {}
    virtual ~ENoModifySizeRights() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INCOMPATIBLE_COMPONENTS error.
class EIncompatibleComponents : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit EIncompatibleComponents( const std::string& name, const std::string& errorOrigin ) : EComponent( "Component " + name + " has been compared with an incompatible type (might differ in components, size, etc.)" , errorOrigin, PROPHANDLING_INCOMPATIBLE_COMPONENTS ) {}
    virtual ~EIncompatibleComponents() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_NO_USER_ALLOCATED_MEMORY error.
class ENoUserAllocatedMemory : public EComponent
//-----------------------------------------------------------------------------
{
public:
    explicit ENoUserAllocatedMemory( const std::string& name, const std::string& errorOrigin ) : EComponent( "Component " + name + " is missing the cfUserAllocatedMemory flag for this operation" , errorOrigin, PROPHANDLING_NO_USER_ALLOCATED_MEMORY ) {}
    virtual ~ENoUserAllocatedMemory() throw() {}
};

//-----------------------------------------------------------------------------
// PROPERTY EXCEPTION CLASSES :
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// \brief A base class for \b mvIMPACT::acquire::Property related exceptions from the property module
/**
 * This exception object will never be raised in the code. It can't be constructed
 * directly but to check for \b mvIMPACT::acquire::Property related errors a catch block for
 * \b mvIMPACT::acquire::EProperty objects can be written to catch all \b mvIMPACT::acquire::Property related
 * exceptions.
 */
class EProperty : public EComponent
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Creates a new \b mvIMPACT::acquire::EProperty object.
    explicit EProperty(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TPROPHANDLING_ERROR errorCode ) : EComponent( errorString, errorOrigin, errorCode ) {}
    virtual ~EProperty() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INVALID_PROP_VALUE error.
class EInvalidValue : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidValue( const std::string& propname, const std::string& errorOrigin ) : EProperty( "Invalid value for property " + propname, errorOrigin, PROPHANDLING_INVALID_PROP_VALUE ) {}
    virtual ~EInvalidValue() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_VAL_ID_OUT_OF_BOUNDS error.
class EValIDOutOfBounds : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EValIDOutOfBounds( const std::string& propname, const std::string& errorOrigin ) : EProperty( "Value ID out of bounds for " + propname, errorOrigin, PROPHANDLING_PROP_VAL_ID_OUT_OF_BOUNDS ) {}
    virtual ~EValIDOutOfBounds() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_TRANSLATION_TABLE_CORRUPTED error.
class ETranslationTableCorrupted : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit ETranslationTableCorrupted( const std::string& propname, const std::string& errorOrigin ) : EProperty( "The translation dictionary of " + propname + " is corrupted", errorOrigin, PROPHANDLING_PROP_TRANSLATION_TABLE_CORRUPTED ) {}
    virtual ~ETranslationTableCorrupted() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_TRANSLATION_TABLE_NOT_DEFINED error.
class ETranslationTableNotDefined : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit ETranslationTableNotDefined( const std::string& propname, const std::string& errorOrigin ) : EProperty( "Translation dictionary has not been defined for property " + propname, errorOrigin, PROPHANDLING_PROP_TRANSLATION_TABLE_NOT_DEFINED ) {}
    virtual ~ETranslationTableNotDefined() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INVALID_PROP_VALUE_TYPE error.
class EInvalidValueType : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidValueType( const std::string& propname, const std::string& errorOrigin ) : EProperty( "Property " + propname + " does not support this value type", errorOrigin, PROPHANDLING_INVALID_PROP_VALUE_TYPE ) {}
    virtual ~EInvalidValueType() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_VAL_TOO_LARGE error.
class EValTooLarge : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EValTooLarge( const std::string& value, const std::string& maxValue, const std::string& propname, const std::string& errorOrigin ) : EProperty( "The value " + value + " is higher than the maximum value(" + maxValue + ") for property " + propname, errorOrigin, PROPHANDLING_PROP_VAL_TOO_LARGE ) {}
    virtual ~EValTooLarge() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_VAL_TOO_SMALL error.
class EValTooSmall : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EValTooSmall( const std::string& value, const std::string& minValue, const std::string& propname, const std::string& errorOrigin ) : EProperty( "The value(" + value + ") is smaller than the minimum value(" + minValue + ") for property " + propname, errorOrigin, PROPHANDLING_PROP_VAL_TOO_SMALL ) {}
    virtual ~EValTooSmall() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_PROP_VALIDATION_FAILED error.
class EValidationFailed : public EProperty
//-----------------------------------------------------------------------------
{
public:
    explicit EValidationFailed( const std::string& propname, const std::string& errorOrigin ) : EProperty( "The assigned value doesn't pass the validation test for property " + propname, errorOrigin, PROPHANDLING_PROP_VALIDATION_FAILED ) {}
    virtual ~EValidationFailed() throw() {}
};

//-----------------------------------------------------------------------------
// PROPERTYLIST EXCEPTION CLASSES :
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// \brief A base class for component list related exceptions from the property module.
/**
 * This exception object will never be raised in the code. It can't be constructed
 * directly but to check for component list related errors a catch block for
 * \b mvIMPACT::acquire::EPropertyList objects can be written to catch all exceptions related
 * to lists and list operations.
 */
class EPropertyList : public EComponent
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Creates a new \b mvIMPACT::acquire::EPropertyList object.
    explicit EPropertyList(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TPROPHANDLING_ERROR errorCode ) : EComponent( errorString, errorOrigin, errorCode ) {}
    virtual ~EPropertyList() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_COMPONENT_NOT_FOUND error.
class EComponentNotFound : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit EComponentNotFound( const std::string& msg, const std::string& errorOrigin ) : EPropertyList( msg, errorOrigin, PROPHANDLING_COMPONENT_NOT_FOUND ) {}
    virtual ~EComponentNotFound() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_LIST_ENTRY_OCCUPIED error.
class EListEntryOccupied : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit EListEntryOccupied( const std::string& errorOrigin ) : EPropertyList( "List entry occupied already", errorOrigin, PROPHANDLING_LIST_ENTRY_OCCUPIED ) {}
    virtual ~EListEntryOccupied() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_COMPONENT_ID_INVALID error.
class EComponentIDInvalid : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit EComponentIDInvalid( const std::string& errorOrigin ) : EPropertyList( "Invalid component ID", errorOrigin, PROPHANDLING_COMPONENT_ID_INVALID ) {}
    virtual ~EComponentIDInvalid() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_LIST_ID_INVALID error.
class EInvalidListID : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidListID( const std::string& errorOrigin ) : EPropertyList( "Invalid proplist ID", errorOrigin, PROPHANDLING_LIST_ID_INVALID ) {}
    virtual ~EInvalidListID() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_LIST_CANT_ACCESS_DATA error.
class ECantAccessData : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit ECantAccessData( const std::string& path, const std::string& errorOrigin ) : EPropertyList( "Cannot access data from " + path, errorOrigin, PROPHANDLING_LIST_CANT_ACCESS_DATA ) {}
    virtual ~ECantAccessData() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_CANT_REGISTER_COMPONENT error.
class ECantRegisterComponent : public EPropertyList
//-----------------------------------------------------------------------------
{
public:
    explicit ECantRegisterComponent( const std::string& errorOrigin ) : EPropertyList( "Cannot register component in the current list.", errorOrigin, PROPHANDLING_CANT_REGISTER_COMPONENT ) {}
    virtual ~ECantRegisterComponent() throw() {}
};

//-----------------------------------------------------------------------------
// METHOD EXCEPTION CLASSES :
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// \brief A base class for \b mvIMPACT::acquire::Method object related exceptions from the property module.
/**
 * This exception object will never be raised in the code. It can't be constructed
 * directly but to check for \b mvIMPACT::acquire::Method object related errors a catch block for
 * \b mvIMPACT::acquire::EMethod objects can be written to catch only \b Method related exceptions.
 */
class EMethod : public EComponent
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Creates a new \b mvIMPACT::acquire::EMethod object.
    explicit EMethod(
        /// [in] The error string.
        const std::string& errorString,
        /// [in] The error origin.
        const std::string& errorOrigin,
        /// [in] The error code.
        TPROPHANDLING_ERROR errorCode ) : EComponent( errorString, errorOrigin, errorCode ) {}
    virtual ~EMethod() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_METHOD_PTR_INVALID error.
class EMethodPtrInvalid : public EMethod
//-----------------------------------------------------------------------------
{
public:
    explicit EMethodPtrInvalid( const std::string& methodname, const std::string& errorOrigin ) : EMethod( "The function pointer for Method " + methodname + " is invalid", errorOrigin, PROPHANDLING_METHOD_PTR_INVALID ) {}
    virtual ~EMethodPtrInvalid() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_METHOD_INVALID_PARAM_LIST error.
class EInvalidParameterList : public EMethod
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidParameterList( const std::string& errorOrigin ) : EMethod( "The functions parameter list is invalid", errorOrigin, PROPHANDLING_METHOD_INVALID_PARAM_LIST ) {}
    virtual ~EInvalidParameterList() throw() {}
};

//-----------------------------------------------------------------------------
// PROPERTYHANDLING EXCEPTION CLASSES :
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_UNSUPPORTED_PARAMETER error.
class EUnsupportedParameter : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EUnsupportedParameter( const std::string& errorOrigin ) : EPropertyHandling( "Unsupported parameter", errorOrigin, PROPHANDLING_UNSUPPORTED_PARAMETER ) {}
    virtual ~EUnsupportedParameter() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_SIZE_MISMATCH error.
class ESizeMismatch : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit ESizeMismatch( const std::string& errorString, const std::string& errorOrigin ) : EPropertyHandling( errorString, errorOrigin, PROPHANDLING_SIZE_MISMATCH ) {}
    virtual ~ESizeMismatch() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_IMPLEMENTATION_MISSING error.
class EImplementationMissing : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EImplementationMissing( const std::string& errorOrigin ) : EPropertyHandling( "This feature has not been implemented so far", errorOrigin, PROPHANDLING_IMPLEMENTATION_MISSING ) {}
    virtual ~EImplementationMissing() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INVALID_INPUT_PARAMETER error.
class EInvalidInputParameter : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidInputParameter( const std::string& errorOrigin ) : EPropertyHandling( "One or more of the input parameters are invalid ( unassigned pointers? )", errorOrigin, PROPHANDLING_INVALID_INPUT_PARAMETER ) {}
    virtual ~EInvalidInputParameter() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INPUT_BUFFER_TOO_SMALL error.
class EInputBufferTooSmall : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EInputBufferTooSmall( const std::string& errorOrigin ) : EPropertyHandling( "The user supplied input buffer was too small for the result", errorOrigin, PROPHANDLING_INPUT_BUFFER_TOO_SMALL ) {}
    virtual ~EInputBufferTooSmall() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_WRONG_PARAM_COUNT error.
class EWrongParamCount : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EWrongParamCount( const std::string& errorOrigin ) : EPropertyHandling( "Wrong parameter count", errorOrigin, PROPHANDLING_WRONG_PARAM_COUNT ) {}
    virtual ~EWrongParamCount() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_UNSUPPORTED_OPERATION error.
class EUnsupportedOperation : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EUnsupportedOperation( const std::string& errorOrigin ) : EPropertyHandling( "This component does not support this operation", errorOrigin, PROPHANDLING_UNSUPPORTED_OPERATION ) {}
    virtual ~EUnsupportedOperation() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_CANT_SERIALIZE_DATA error.
class ECantSerializeData : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit ECantSerializeData( const std::string& listname, const std::string& errorOrigin ) : EPropertyHandling( "No serialize rights for list " + listname, errorOrigin, PROPHANDLING_CANT_SERIALIZE_DATA ) {}
    virtual ~ECantSerializeData() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_INVALID_FILE_CONTENT error.
class EInvalidFileContent : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit EInvalidFileContent( const std::string& path, const std::string& errorOrigin ) : EPropertyHandling( "The file " + path + " does not contain valid data for this operation", errorOrigin, PROPHANDLING_INVALID_FILE_CONTENT ) {}
    virtual ~EInvalidFileContent() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief An exception thrown in case of a \b mvIMPACT::acquire::PROPHANDLING_CANT_ALLOCATE_LIST error.
class ECantAllocateNewList : public EPropertyHandling
//-----------------------------------------------------------------------------
{
public:
    explicit ECantAllocateNewList( const std::string& errorOrigin ) : EPropertyHandling( "Cannot allocate a new list until an old one has been deleted", errorOrigin, PROPHANDLING_CANT_ALLOCATE_LIST ) {}
    virtual ~ECantAllocateNewList() throw() {}
};

//-----------------------------------------------------------------------------
/// \brief A factory class to raise mvIMPACT acquire related exceptions
/**
 * This class contains a collection of static member functions that are used inside the
 * interface to raise the appropriate exception in case of an error
 */
class ExceptionFactory
//-----------------------------------------------------------------------------
{
public:
    /// \brief Raises an exception from an error code, its origin, a string with additional information and an optional component handle
    static void raiseException(
        /// [in] The name of the function this exception was raised from.
        const char* pFunctionName,
        /// [in] The line number in the source this exception was raised from.
        int lineNumber,
        /// [in] The error code.
        int errorCode,
        /// [in] A handle to an object.
        HOBJ objectHandle = INVALID_ID,
        /// [in] A string with additional information.
        const std::string& additionalInfo = "" );
};
#endif // #ifndef WRAP_DOTNET

#ifdef DOXYGEN_SHOULD_SKIP_THIS
#   define MVIMPACT_H_ // force doxygen to include mvIMPACT related functions
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

//-----------------------------------------------------------------------------
/// \brief A base class to implement access to internal driver objects.
/**
 * Instances of this class can't be constructed directly.
 */
class ComponentAccess
//-----------------------------------------------------------------------------
{
    friend class ComponentLocatorBase;
protected:
    /// \brief An internal constant that defines by which factor dynamic buffers will grow when the current size is not sufficient.
    enum { BUFFER_INCREMENT_FACTOR = 6 };
    /// \brief A unique identifier for the internal driver object referenced by this instance of \b mvIMPACT::acquire::ComponentAccess
    HOBJ m_hObj;
    /// \brief A helper function to query certain component related string parameters.
    /**
     * This function might throw an exception, if an invalid parameter has been queried.
     * \return A string containing the data to be queried.
     */
    std::string compGetStringParam( /// The type of the parameter to read
        TOBJ_StringQuery query,
        /// An additional parameter
        int param1 = 0,
        /// An additional parameter
        int param2 = 0 ) const
    {
        char* pStr = 0;
        TPROPHANDLING_ERROR result;
        std::string resultString;
        result = OBJ_GetSWithInplaceConstruction( m_hObj, query, &pStr, stringAllocator, param1, param2 );
        if( pStr )
        {
            resultString = std::string( pStr );
        }
        delete [] pStr;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return resultString;
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::ComponentAccess object to a driver object.
    explicit ComponentAccess(   /// A valid handle to a component object
        HOBJ hObj ) : m_hObj( hObj )
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_CheckHandle( m_hObj, hcmOwnerList ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
    }
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::ComponentAccess object.
    explicit ComponentAccess( void ) : m_hObj( INVALID_ID ) {}
    /// \brief An internal helper function for fast string allocation.
    static char* stringAllocator( const char* pBuf, size_t reqBufSize )
    {
        char* pStr = new char[reqBufSize];
#if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
        errno_t result = strcpy_s( pStr, reqBufSize, pBuf );
        result = result; // remove release build warning
        assert( result == 0 );
        return pStr;
#else
        return strcpy( pStr, pBuf );
#endif // #if defined(_MSC_VER) && (_MSC_VER >= 1400)
    }
public:
    virtual ~ComponentAccess( void ) {}
#ifndef WRAP_PYTHON
    /// \brief Allows implicit conversion to a HOBJ.
    operator HOBJ() const
    {
        return m_hObj;
    }
#endif // #ifndef WRAP_PYTHON (In Python, be explicit: use method hObj(); Python variables lack a type specifier, so a conversion operator is useless for variable assignment anyhow)
    /// \brief Returns the current changed counter for the component referenced by this object.
    /**
     *  This changed counter is incremented internally each time the component is modified.
     *  To check if this component has been modified since the last time, this check has been
     *  performed, keep track of the last return value of this function and compare
     *  it with the new value. This can be helpful e.g. to keep a GUI up to date. The value returned
     *  by this function will always be larger than or equal to the value returned by
     *  \b mvIMPACT::acquire::ComponentAccess::changedCounterAttr (except in case of a
     *  wrap around) when called at the same time for the same object as it's always incremented
     *  when the component has been modified in
     *  any way while the latter one will only be incremented if the attributes (e.g. the
     *  flags) but \b NOT if e.g. the value(s) of a property has been modified.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *
     * \code
     *  // EXAMPLE
     *  //-----------------------------------------------------------------------------
     *  struct ComponentRef
     *  //-----------------------------------------------------------------------------
     *  {
     *    mvIMPACT::acquire::Component* m_pc;
     *    unsigned int m_lastChangedCount;
     *    ComponentRef( mvIMPACT::acquire::Component* pc ) : m_pc(pc), m_lastChangedCount(0) {}
     *  };
     *
     *  //-----------------------------------------------------------------------------
     *  void fn( ComponentRef& cr )
     *  //-----------------------------------------------------------------------------
     *  {
     *    if( cr.m_pc )
     *    {
     *      unsigned int currentChangedCount = cr.m_pc->changedCounter();
     *      if( currentChangedCount != cr.m_lastChangedCount ) )
     *      {
     *        // something has happened since the last check.
     *        doWhatNeedsToBeDone();
     *        // and remember the current changed counter
     *        cr.m_lastChangedCount = currentChangedCount;
     *      }
     *    }
     *  }
     * \endcode
     *  \endif
     *  \return The current changed counter of this object.
     */
    unsigned int changedCounter( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int changedCounter;
        if( ( result = OBJ_GetChangedCounter( m_hObj, &changedCounter ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return changedCounter;
    }
    /// \brief Returns the current attribute changed counter for the component referenced by this object.
    /**
     * This changed counter is incremented internally each time the components attributes have been modified.
     * To check if this components attributes have been modified since the last time, this check has been
     * performed, keep track of the last return value of this function and compare
     * it with the new value. This can be helpful e.g. to keep a GUI up to date.
     *
     * \note
     * Attributes changes are e.g. a modification to a properties translation dictionary,
     * but \b NOT a properties value. Because of this the value returned by this function
     * will always be less or equal than the value returned by the function
     * \b mvIMPACT::acquire::ComponentAccess::changedCounter (except in case of a
     * wrap around) when called at the same time for the same object.
     * \sa \b mvIMPACT::acquire::ComponentAccess::changedCounter
     * \return The current attributes changed counter of this object.
     */
    unsigned int changedCounterAttr( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int changedCounter;
        if( ( result = OBJ_GetChangedCounterAttr( m_hObj, &changedCounter ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return changedCounter;
    }
    /// \brief Returns a unique identifier for the component referenced by this object.
    /**
     *  \return A unique identifier for the component referenced by this object.
     */
    HOBJ hObj( void ) const
    {
        return m_hObj;
    }
    /// \brief Returns the name of the component referenced by this object.
    /**
     *  \return The name of the component referenced by this object.
     */
    std::string name( void ) const
    {
        return compGetStringParam( sqObjName );
    }
    /// \brief Returns the display name of the component referenced by this object.
    /**
     *  \return The display name of the component referenced by this object. This might be an empty string if no display name has been specified.
     */
    std::string displayName( void ) const
    {
        return compGetStringParam( sqObjDisplayName );
    }
};

//-----------------------------------------------------------------------------
/// \brief A base class to implement access to internal driver components.
/**
 *  Objects of this class can be constructed directly even if nothing is known about the type
 *  of driver object we are referring to. This class acts as a base class to provide
 *  access to internal properties, methods and component lists offered by the driver.
 *
 *  This object can be used to navigate through component lists of
 *  unknown content.
 *
 *  Consider the following structure:
 *
 * \code
 *  LA
 *  |-LB
 *  |-LC
 *  |  |-PE
 *  |  |-PF
 *  |  |-PG
 *  |-PD
 * \endcode
 *
 *  Where the prefix 'L' means this is a list, 'P' that this is a property.
 *  Assuming that we have and iterator referencing list 'C', calling \b mvIMPACT::acquire::ComponentIterator::firstChild
 *  e.g. would return a new iterator referencing object 'PE', while calling \b mvIMPACT::acquire::ComponentIterator::nextSibling
 *  would have returned a reference to 'PD' and \b mvIMPACT::acquire::ComponentIterator::parent would have
 *  returned a reference to object 'LA'.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  \b "EXAMPLE 1":
 *
 *  A new \b mvIMPACT::acquire::ComponentIterator is created with the \a ID of list 'C':
 * \code
 *  ComponentIterator it(ID_of_list_C);
 *  it = it.firstChild();    // now we are referencing 'PE'
 *  it = it.lastSibling();   // moves to 'PG'
 *  it = it.firstSibling();  // moves back to PE'
 *  it = it.nextSibling();   // moves to 'PF'
 *  it = it.firstSibling();  // moves back to PE'
 *  it = it.parent();        // we are referencing 'LC' again
 * \endcode
 *
 *  \b "EXAMPLE 2":
 *
 *  Iterate over a complete list including sub lists. This will result in a list
 *  of all lists and properties that reside in the list the iterator currently
 *  is moving through to be written to the standard output. The name of the component
 *  and every parent component will be printed into the standard output:
 *
 * \code
 *  //-----------------------------------------------------------------------------
 *  void ParseList( ComponentIterator iter, const string& path = "" )
 *  //-----------------------------------------------------------------------------
 *  {
 *    while( iter.isValid() )
 *    {
 *      if( iter.isVisible() )
 *      {
 *        if( iter.isList() )
 *        {
 *          // do some list specific stuff
 *          cout << "List     " << path << iter.name() << "/" << endl;
 *          // move down into the list and append its name to the path
 *          ParseList( iter.firstChild(), path + iter.name() + "/" );
 *        }
 *        else if( iter.isProp() )
 *        {
 *          // do property specific stuff e.g. read the value
 *          Property prop(iter);
 *          cout << "Property " << path << prop.name() << "(value(s): ";
 *          unsigned int valCount = prop.valCount();
 *          for( unsigned int i=0; i<valCount; i++ )
 *          {
 *            cout << prop.readS();
 *            if( i < valCount - 1 )
 *            {
 *              cout << ", ";
 *            }
 *          }
 *          cout << ")" << endl;
 *        }
 *      }
 *      ++iter;
 *    }
 *  }
 *
 *  //-----------------------------------------------------------------------------
 *  int main( int argc, char* argv[] )
 *  //-----------------------------------------------------------------------------
 *  {
 *    ComponentList baselist;
 *    // ....
 *    ComponentIterator it(baselist);
 *    ParseList(it);
 *    // ....
 *    return 0;
 *  }
 * \endcode
 *  \elseif DOXYGEN_NET_DOCUMENTATION
 *
 *  A new \b mvIMPACT::acquire::ComponentIterator is created with the \a ID of list 'C':
 * \code
 *  ComponentIterator it = new ComponentIterator(ID_of_list_C);
 *  it = it.firstChild();   // now we are referencing 'PE'
 *  it = it.lastSibling();  // moves to 'PG'
 *  it = it.firstSibling(); // moves back to PE'
 *  it = it.nextSibling();  // moves to 'PF'
 *  it = it.firstSibling(); // moves back to PE'
 *  it = it.parent();       // we are referencing 'LC' again
 * \endcode
 *
 *  \b EXAMPLE 2":
 *
 *  Iterate over a complete list including sub lists. This will result in a list
 *  of all lists and properties that reside in the list the iterator currently
 *  is moving through to be written to the standard output. The name of the component
 *  and every parent component will be printed into the standard output:
 *
 * \code
 *  static void outputPropData( Property prop )
 *  {
 *    // do property specific stuff e.g. read the value
 *    Console.Write( prop.name() + "(value(s): " );
 *    uint valCount = prop.valCount();
 *    for( uint i=0; i<valCount; i++ )
 *    {
 *      Console.Write( prop.readS() );
 *      if( i < valCount - 1 )
 *      {
 *        Console.Write( ", " );
 *      }
 *    }
 *    Console.WriteLine( ")" );
 *  }
 *
 *  static void ParseList( ComponentIterator iter, String path )
 *  {
 *    while( iter.valid )
 *    {
 *      if( iter.visible )
 *      {
 *        if( iter.list )
 *        {
 *          // do some list specific stuff
 *          Console.WriteLine( "List     " + path + iter.name() );
 *          // move down into the list and append its name to the path
 *          ParseList( iter.firstChild(), path + iter.name() + "/" );
 *        }
 *        else if( iter.prop )
 *        {
 *          Console.Write( "Property " );
 *          Console.Write( path );
 *          TComponentType type = iter.type();
 *          if( type == TComponentType.ctPropInt )
 *          {
 *            outputPropData( new PropertyI(iter.hObj()) );
 *          }
 *          else if( type == TComponentType.ctPropFloat )
 *          {
 *            outputPropData( new PropertyF(iter.hObj()) );
 *          }
 *          else if( type == TComponentType.ctPropString )
 *          {
 *            outputPropData( new PropertyS(iter.hObj()) );
 *          }
 *          else
 *          {
 *            Console.WriteLine( "This property(" + iter.name() + ") is currently unsupported under .NET" );
 *          }
 *        }
 *      }
 *      iter.nextSibling();
 *    }
 *  }
 *
 *  [STAThread]
 *  static void Main(string[] args)
 *  {
 *    try
 *    {
 *      ComponentList baselist = new ComponentList;
 *      // ...
 *      ComponentIterator it = new ComponentIterator(baselist);
 *      ParseList(it);
 *      // ...
 *    }
 *    catch(ImpactException e)
 *    {
 *      Console.WriteLine(e.Message);
 *    }
 *  }
 * \endcode
 * \endif
 */
class Component : public ComponentAccess
//-----------------------------------------------------------------------------
{
private:
    //-----------------------------------------------------------------------------
    template<typename _FnQuery> size_t queryFeatureCount( _FnQuery pFnQuery ) const
    //-----------------------------------------------------------------------------
    {
        TPROPHANDLING_ERROR result;
        size_t featureCount = 0;
        if( ( result = pFnQuery( m_hObj, 0, 0, &featureCount ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return featureCount;
    }
    //-----------------------------------------------------------------------------
    template<typename _FnQuery> size_t queryFeatures( std::vector<Component>& v, _FnQuery pFnQuery ) const
    //-----------------------------------------------------------------------------
    {
        TPROPHANDLING_ERROR result;
        v.clear();
        size_t featureCount = queryFeatureCount( pFnQuery );
        if( featureCount > 0 )
        {
            std::vector<HOBJ> buf;
            buf.resize( featureCount );
            if( ( result = pFnQuery( m_hObj, 0, &( *( buf.begin() ) ), &featureCount ) ) != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
            v.resize( featureCount );
            for( size_t i = 0; i < featureCount; i++ )
            {
                v[i] = Component( buf[i] );
            }
        }
        return static_cast<size_t>( v.size() );
    }
    //-----------------------------------------------------------------------------
    template<typename _FnQuery> HOBJ queryFeature( _FnQuery pFnQuery, int index ) const
    //-----------------------------------------------------------------------------
    {
        TPROPHANDLING_ERROR result;
        HOBJ hObj;
        size_t featureCount = 1;
        if( ( result = pFnQuery( m_hObj, index, &hObj, &featureCount ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return hObj;
    }
    //-----------------------------------------------------------------------------
    template<typename _FnQuery> std::string queryAsString( _FnQuery pFnQuery ) const
    //-----------------------------------------------------------------------------
    {
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pBuf = new char[bufSize];
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        while( ( result = pFnQuery( m_hObj, pBuf, bufSize ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            delete [] pBuf;
            bufSize *= BUFFER_INCREMENT_FACTOR;
            pBuf = new char[bufSize];
        }
        std::string value( pBuf );
        delete [] pBuf;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return value;
    }
public:
    /// \brief Constructs a new access object to a driver object.
    explicit Component(
        /// [in] A valid handle to a component object
        HOBJ hObj ) : ComponentAccess( hObj ) {}
    /// \brief Constructs a new unbound access object.
    explicit Component( void ) {}
#if !defined(WRAP_DOTNET) && !defined(WRAP_PYTHON)
    /// \brief Moves to the next sibling(the next feature in the current list of features).
    /**
     *  \return A self reference
     */
    Component& operator++( void ) // prefix
    {
        TPROPHANDLING_ERROR result;
        HOBJ nextSibling;
        if( ( result = OBJ_GetNextSibling( m_hObj, &nextSibling ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        m_hObj = nextSibling;
        return *this;
    }
    /// \brief Moves to the next sibling(the next feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::Component object
     */
    Component operator++( int ) // postfix
    {
        Component temp( *this );
        ++*this;
        return temp;
    }
#endif // #if !defined(WRAP_DOTNET) && !defined(WRAP_PYTHON)
#ifndef WRAP_DOTNET
    /// \brief Moves to the first sibling(the first feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::Component object
     */
    Component firstSibling( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetFirstSibling( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? Component( hNew ) : Component();
    }
    /// \brief Moves to the last sibling(the last feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::Component object
     */
    Component lastSibling( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetLastSibling( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? Component( hNew ) : Component();
    }
    /// \brief Moves to the next sibling(the next feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::Component object
     */
    Component nextSibling( void )
    {
        return ++*this;
    }
    /// \brief Moves to the first child of this component(moves down one level).
    /**
     * Calling this function will only succeed, if the current \b mvIMPACT::acquire::Component
     * references a list.
     *
     * \return A new \b mvIMPACT::acquire::Component object
     */
    Component firstChild( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetFirstChild( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? Component( hNew ) : Component();
    }
    /// \brief Moves to the parent of this component(moves up one level).
    /**
     *  \return A new \b mvIMPACT::acquire::Component object
     */
    Component parent( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetParent( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? Component( hNew ) : Component();
    }
#endif // #ifndef WRAP_DOTNET
    /// \brief Returns a string containing general information about the component referenced by this object.
    /**
     *  \return A string containing general information about the component referenced by this object.
     */
    std::string docString( void ) const
    {
        return compGetStringParam( sqObjDocString );
    }
    /// \brief Returns the flags associated with this component.
    /**
     *  \return The flags associated with this component.
     */
    TComponentFlag flags( void ) const
    {
        TPROPHANDLING_ERROR result;
        TComponentFlag flags;
        if( ( result = OBJ_GetFlags( m_hObj, &flags ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return flags;
    }
    /// \brief Returns the flags associated with this component as a string.
    /**
     *  \return The flags associated with this component as a string.
     */
    std::string flagsAsString(
        /// [in] A user definable string to separate the individual flags.
        ///
        /// The default value is ' | ' resulting in the string to look e.g.
        /// like this: <b>'cfWriteAccess | cfReadAccess'</b>
        const std::string& separator = " | " ) const
    {
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pBuf = new char[bufSize];
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        while( ( result = OBJ_GetFlagsS( m_hObj, separator.c_str(), pBuf, bufSize ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            delete [] pBuf;
            bufSize *= BUFFER_INCREMENT_FACTOR;
            pBuf = new char[bufSize];
        }
        std::string value( pBuf );
        delete [] pBuf;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return value;
    }
    /// \brief Checks if this component is currently referencing the default for this component.
    /**
     *  This function will return true only for derived components that have not been modified.
     *  \return
     *  - true if the component is currently set to its default value
     *  - false otherwise.
     */
    bool isDefault( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int isDefault;
        if( ( result = OBJ_IsDefault( m_hObj, &isDefault ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return isDefault != 0;
    }
    /// \brief Checks if this component is of type \b mvIMPACT::acquire::ComponentList.
    /**
     *  \return
     *  - true if the component references a list
     *  - false otherwise
     */
    bool isList( void ) const
    {
        return ( type() & ctList ) != 0;
    }
    /// \brief Checks if this component is of type \b mvIMPACT::acquire::Method.
    /**
     *  \return
     *  - true if the component references a method
     *  - false otherwise
     */
    bool isMeth( void ) const
    {
        return ( type() & ctMeth ) != 0;
    }
    /// \brief Checks if this component is of type \b mvIMPACT::acquire::Property or a derived type.
    /**
     *  \return
     *  - true if the component references a property
     *  - false otherwise
     */
    bool isProp( void ) const
    {
        return ( type() & ctProp ) != 0;
    }
    /// \brief Checks if the internal component referenced by this object is still valid.
    /**
     *  This function can be used to verify whether a referenced component
     *  is still valid or not. When e.g. referencing a driver property after
     *  \b mvIMPACT::acquire::Device::close has been called this function would return false.
     *  Calling any other function that tries to access the referenced component
     *  in that case would raise an exception.
     *  \return
     *  - true if this object currently references a valid component
     *  - false otherwise.
     */
    bool isValid( void ) const
    {
        return OBJ_CheckHandle( m_hObj, hcmFull ) == PROPHANDLING_NO_ERROR;
    }
    /// \brief Checks if the component is currently shadowed due to a settings made elsewhere or not.
    /**
     * Settings applied to certain components might affect the behaviour of others. For example
     * an activated automatic gain control might shadow the value written to the
     * gain property by the user as the gain is calculated internally. In order to
     * check if modifying the actual component will affect the behaviour of the
     * system this function may be used. When it returns \a true, the \b mvIMPACT::acquire::Component
     * will have an impact on the system, if \a false is returned, the feature might be
     * modified, but this will currently NOT influence the acquisition process or the overall behaviour
     * of the device or driver.
     *
     * This is what is called visibility. The user still might modify or read the
     * current \b mvIMPACT::acquire::Component when it's not visible however the actual data will
     * be used only if the \b Component is visible (\b mvIMPACT::acquire::cfInvisible
     * must \b NOT be set).
     *
     * The visibility of a \b mvIMPACT::acquire::Component object will change only if other
     * \b mvIMPACT::acquire::Component objects are modified and \b NEVER when a program
     * runs but does not change any \b mvIMPACT::acquire::Component.
     */
    bool isVisible( void ) const
    {
        return ( flags() & cfInvisible ) == 0;
    }
    /// \brief Checks if the caller has write/modify access to the component.
    /**
     *  \return
     *  - true if the caller is allowed to call write/modify operation for this component.
     *  - false otherwise.
     */
    bool isWriteable( void ) const
    {
        return ( flags() & cfWriteAccess ) != 0;
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Restores the default for the referenced component.
    /**
     *  Calling this function will restore the default value for the
     *  component referenced by this object.
     *
     *  If this function is called for an object of type \b mvIMPACT::acquire::ComponentList
     *  every component in that list is restored to the default value.
     *
     *  \note The caller must have the right to modify the component. Otherwise
     *  an exception will be thrown.
     */
    void
#else
    /// \brief Restores the default for the referenced component.
    /**
     * Calling this function will restore the default value for the
     * component referenced by this object.
     *
     * If this function is called for an object of type \b mvIMPACT::acquire::ComponentList
     * every component in that list is restored to the default value.
     *
     * \note The caller must have the right to modify the component. Otherwise
     * an exception will be thrown.
     *
     * \return A const reference to the component.
     */
    const Component&
#endif // #if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    restoreDefault( void ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_RestoreDefault( m_hObj ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif // #ifndef DOTNET_ONLY_CODE
    }
    /// \brief Returns the type of the referenced component.
    /**
     *  \return The type of the referenced component.
     */
    TComponentType type( void ) const
    {
        TPROPHANDLING_ERROR result;
        TComponentType type;
        if( ( result = OBJ_GetType( m_hObj, &type ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return type;
    }
    /// \brief Returns the type of the component referenced by \a hObj.
    /**
     *  \return The type of the component referenced by \a hObj.
     */
    static TComponentType type(
        /// [in] The component the type shall be retrieved for
        HOBJ hObj )
    {
        return Component( hObj ).type();
    }
    /// \brief Returns the type of the referenced component as a string.
    /**
     *  \return The type of the referenced component as a string.
     */
    std::string typeAsString( void ) const
    {
        return queryAsString( OBJ_GetTypeS );
    }
    /// \brief Returns the recommended visibility for this component.
    /**
     * This visibility can be used e.g. to develop a GUI that allows to display the
     * crucial subset of features only.
     * \return The recommended visibility for this component.
     */
    TComponentVisibility visibility( void ) const
    {
        TPROPHANDLING_ERROR result;
        TComponentVisibility visibility;
        if( ( result = OBJ_GetVisibility( m_hObj, &visibility ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return visibility;
    }
    /// \brief Returns the recommended visibility of the referenced component as a string.
    /**
     *  \return The recommended visibility of the referenced component as a string.
     */
    std::string visibilityAsString( void ) const
    {
        return queryAsString( OBJ_GetVisibilityS );
    }
    /// \brief Returns the recommended visibility converted to a string.
    /**
     *  \return The recommended visibility converted to a string.
     */
    static std::string visibilityAsString(
        /// [in] The visibility to query the string representation for
        TComponentVisibility visibility )
    {
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pBuf = new char[bufSize];
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        while( ( result = OBJ_VisibilityToString( visibility, pBuf, bufSize ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            delete [] pBuf;
            bufSize *= BUFFER_INCREMENT_FACTOR;
            pBuf = new char[bufSize];
        }
        std::string value( pBuf );
        delete [] pBuf;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID );
        }
        return value;
    }
    /// \brief Returns the number of features selected by the current one.
    /**
     * \sa
     * \b mvIMPACT::acquire::Component::selectedFeatures, \n
     * \b mvIMPACT::acquire::Component::selectedFeature
     * \return The number of features selected by the current one.
     */
    unsigned int selectedFeatureCount( void ) const
    {
        return static_cast<unsigned int>( queryFeatureCount( OBJ_GetSelectedFeatures ) );
    }
    /// \brief Returns the number of features selecting the current one.
    /**
     * \sa
     * \b mvIMPACT::acquire::Component::selectingFeatures, \n
     * \b mvIMPACT::acquire::Component::selectingFeature
     * \return The number of features selecting the current one.
     */
    unsigned int selectingFeatureCount( void ) const
    {
        return static_cast<unsigned int>( queryFeatureCount( OBJ_GetSelectingFeatures ) );
    }
#ifndef WRAP_DOTNET
    /// \brief Retrieves the list of components that are selected by the current one.
    /**
     * This function retrieves the list of components that are selected by
     * the current one. This information is mainly useful for GUI applications that want to arrange
     * features in a way that dependencies between features can easily been spotted.
     *
     * When a component 'selects' other components, this indicates that selected components may change whenever
     * the selecting component changes. An example for a selector might be a property defining the index within
     * a LUT while the value of a particular LUT entry could be a selected feature. Assuming 2 properties
     * \a LUTIndex and \a LUTValue then changing \a LUTIndex will invalidate and possibly change \a LUTValue.
     *
     * \sa
     * \b mvIMPACT::acquire::Component::selectedFeatureCount, \n
     * \b mvIMPACT::acquire::Component::selectedFeature
     * \return The number of features selected by the current one.
     */
    unsigned int selectedFeatures(
        /// [out] An array that will retrieve the list of components that are selected by the current one.
        std::vector<Component>& v ) const
    {
        return static_cast<unsigned int>( queryFeatures( v, OBJ_GetSelectedFeatures ) );
    }
    /// \brief Retrieves the list of components that are selecting the current one.
    /**
     * This function retrieves the list of components that are selecting
     * the current one. This information is mainly useful for GUI applications that want to arrange
     * features in a way that dependencies between features can easily been spotted.
     *
     * When a component 'selects' other components, this indicates that selected components may change whenever
     * the selecting component changes. An example for a selector might be a property defining the index within
     * a LUT while the value of a particular LUT entry could be a selected feature. Assuming 2 properties
     * \a LUTIndex and \a LUTValue then changing \a LUTIndex will invalidate and possibly change \a LUTValue.
     *
     * \sa
     * \b mvIMPACT::acquire::Component::selectingFeatureCount, \n
     * \b mvIMPACT::acquire::Component::selectingFeature
     * \return The number of features selecting the current one.
     */
    unsigned int selectingFeatures(
        /// [out] An array that will retrieve the list of components that are selecting the current one.
        std::vector<Component>& v ) const
    {
        return static_cast<unsigned int>( queryFeatures( v, OBJ_GetSelectingFeatures ) );
    }
#endif // #ifndef WRAP_DOTNET
    /// \brief Retrieves a component that is selected by the current one.
    /**
     * This function retrieves a component that is selected by the current one.
     * This information is mainly useful for GUI applications that want to arrange
     * features in a way that dependencies between features can easily been spotted.
     *
     * When a component 'selects' other components, this indicates that selected components may change whenever
     * the selecting component changes. An example for a selector might be a property defining the index within
     * a LUT while the value of a particular LUT entry could be a selected feature. Assuming 2 properties
     * \a LUTIndex and \a LUTValue then changing \a LUTIndex will invalidate and possibly change \a LUTValue.
     *
     * \if DOXYGEN_CPP_DOCUMENTATION
     * C++ offers the more efficient function \b mvIMPACT::acquire::Component::selectedFeatures to obtain this
     * information.
     * \endif
     *
     * To find out how many \b mvIMPACT::acquire::Component objects are selected by the current one call
     * \b mvIMPACT::acquire::Component::selectedFeatureCount. This value minus 1 will also be the
     * max. value for \a index.
     * \sa
     * \b mvIMPACT::acquire::Component::selectedFeatureCount, \n
     * \b mvIMPACT::acquire::Component::selectedFeatures
     * \return A \b mvIMPACT::acquire::Component that is selected by the current one.
     */
#ifndef DOTNET_ONLY_CODE
    Component selectedFeature(
        /// [in] The index for the component to query.
        unsigned int index ) const
    {
        return Component( queryFeature( OBJ_GetSelectedFeatures, static_cast<int>( index ) ) );
    }
#else
    HOBJ selectedFeature(
        /// [in] The index for the component to query.
        unsigned int index ) const
    {
        return queryFeature( OBJ_GetSelectedFeatures, static_cast<int>( index ) );
    }
#endif // #ifndef DOTNET_ONLY_CODE
    /// \brief Retrieves a component that is selecting the current one.
    /**
     * This function retrieves a component that is selecting the current one.
     * This information is mainly useful for GUI applications that want to arrange
     * features in a way that dependencies between features can easily been spotted.
     *
     * When a component 'selects' other components, this indicates that selected components may change whenever
     * the selecting component changes. An example for a selector might be a property defining the index within
     * a LUT while the value of a particular LUT entry could be a selected feature. Assuming 2 properties
     * \a LUTIndex and \a LUTValue then changing \a LUTIndex will invalidate and possibly change \a LUTValue.
     *
     * \if DOXYGEN_CPP_DOCUMENTATION
     * C++ offers the more efficient function \b mvIMPACT::acquire::Component::selectingFeatures to obtain this
     * information.
     * \endif
     *
     * To find out how many \b mvIMPACT::acquire::Component objects are selecting the current one call
     * \b mvIMPACT::acquire::Component::selectingFeatureCount. This value minus 1 will also be the
     * max. value for \a index.
     * \sa
     * \b mvIMPACT::acquire::Component::selectingFeatureCount, \n
     * \b mvIMPACT::acquire::Component::selectingFeatures
     * \return A \b mvIMPACT::acquire::Component that is selecting the current one.
     */
#ifndef DOTNET_ONLY_CODE
    Component selectingFeature(
        /// [in] The index for the component to query.
        unsigned int index ) const
    {
        return Component( queryFeature( OBJ_GetSelectingFeatures, static_cast<int>( index ) ) );
    }
#else
    HOBJ selectingFeature(
        /// [in] The index for the component to query.
        unsigned int index ) const
    {
        return queryFeature( OBJ_GetSelectingFeatures, static_cast<int>( index ) );
    }
#endif // #ifndef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A simple helper class to wrap the creation of a callback object.
/**
 * Applications need to derive from this class and must re-implement the function
 * \b mvIMPACT::acquire::ComponentCallback::execute.
 */
class ComponentCallback
//-----------------------------------------------------------------------------
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct CallbackUserData
            //-----------------------------------------------------------------------------
    {
        void* pUserData_;
        ComponentCallback* pCallback_;
        explicit CallbackUserData() : pUserData_( 0 ), pCallback_( 0 ) {}
    };
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        CallbackHandle handle_;
        CallbackUserData callbackUserData_;
        std::set<HOBJ> objectsRegistered_;
        int m_refCnt;
        ReferenceCountedData() : handle_( 0 ), callbackUserData_(), objectsRegistered_(), m_refCnt( 1 ) {}
    }* m_pRefData;
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

    //-----------------------------------------------------------------------------
    static void DMR_CALL myCallback( HOBJ hObj, void* pUserData )
    //-----------------------------------------------------------------------------
    {
        Component c( hObj );
        CallbackUserData* p = reinterpret_cast<CallbackUserData*>( pUserData );
        p->pCallback_->execute( c, p->pUserData_ );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            OBJ_DeleteCallback( m_pRefData->handle_ );
            delete m_pRefData;
        }
    }
public:
    /// \brief Creates a new \b mvIMPACT::acquire::ComponentCallback instance.
    explicit ComponentCallback(
        /// [in] A pointer to user specific data bound to this callback. This data will
        /// be passed back to the user whenever the callback function gets executed and can be used to attach
        /// custom data to the callback function.
        void* pUserData = 0 ) : m_pRefData( new ReferenceCountedData() )
    {
        m_pRefData->callbackUserData_.pUserData_ = pUserData;
        m_pRefData->callbackUserData_.pCallback_ = this;
        TPROPHANDLING_ERROR result = OBJ_CreateCallback( ctOnChanged, myCallback, &m_pRefData->callbackUserData_, &m_pRefData->handle_ );
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID );
        }
    }
    /// \brief Copy constructor
    /**
     * Creates a new object from an existing device object. Keep in mind that this new object
     * will provide access to the very same. This constructor
     * is only provided for internal reference counting to guarantee correct operation of the
     * objects of this class under all platforms and languages.
     */
    explicit ComponentCallback( const ComponentCallback& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor
    /**
     * This will automatically unregister all components still referencing this callback and will
     * free all internal resources.
     */
    virtual ~ComponentCallback()
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::ComponentCallback objects
    ComponentCallback& operator=( const ComponentCallback& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++( m_pRefData->m_refCnt );
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Checks if a component is currently registered for this callback or not.
    /**
     * \return
     *  - true if the component currently is registered for this callback
     *  - false otherwise
     */
    bool isComponentRegistered(
        /// [in] The compoent that shall be tested against this callback
        Component c ) const
    {
        return m_pRefData->objectsRegistered_.find( c.hObj() ) != m_pRefData->objectsRegistered_.end();
    }
    /// \brief Registers a component for this callback.
    /**
     * \return
     *  - true if the component could be registered successfully for this callback.
     *  - false otherwise
     */
    bool registerComponent(
        /// [in] The component to register for this callback
        Component c )
    {
        if( isComponentRegistered( c ) )
        {
            return false;
        }
        TPROPHANDLING_ERROR result = OBJ_AttachCallback( c.hObj(), m_pRefData->handle_ );
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID );
        }
        m_pRefData->objectsRegistered_.insert( c.hObj() );
        return true;
    }
    /// \brief Unregisters a component for this callback.
    /**
     * \return
     *  - true if the component could be unregistered successfully for this callback.
     *  - false otherwise
     */
    bool unregisterComponent(
        /// [in] The component to unregister from this callback
        Component c )
    {
        std::set<HOBJ>::iterator it = m_pRefData->objectsRegistered_.find( c.hObj() );
        if( it == m_pRefData->objectsRegistered_.end() )
        {
            return false;
        }
        TPROPHANDLING_ERROR result = OBJ_DetachCallback( c.hObj(), m_pRefData->handle_ );
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID );
        }
        m_pRefData->objectsRegistered_.erase( *it );
        return true;
    }
    /// \brief The callback function.
    /**
     * Re-implement this function in a derived class in order to get the desired behaviour.
     *
     * \note
     * Please note that this function might be executed from \b ANY thread context, which is
     * most likely not the same as used by the application thus appropriate mechanisms to ensure
     * correct execution must be implemented by an application(e.g. GUI applications might send an
     * event to the main thread instead of directly accessing GUI elements).
     */
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    virtual void execute(
        /// [in] The component that did cause the callback to be executed
        Component& c,
        /// [in] A pointer to user specific data that was bound to this callback instance
        /// upon construction
        void* pUserData ) {}
#else
    virtual void execute(
        /// [in] The component that did cause the callback to be executed
        Component& /*c*/,
        /// [in] A pointer to user specific data that was bound to this callback instance
        /// upon construction
        void* /*pUserData*/ ) {}
#endif // #if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
};

//-----------------------------------------------------------------------------
/// \brief A base class to locate components within the driver.
/**
 * Every driver will offer a set of properties, methods and component lists.
 * \b mvIMPACT::acquire::Property objects contain data such as the current gain in dB, the state of a
 * digital input, etc.. \b mvIMPACT::acquire::Method objects can be executed like a normal function and
 * \b mvIMPACT::acquire::ComponentList objects are used to group certain
 * objects together to form a logical unit.
 *
 * When it's necessary to locate one or more of these objects without knowing
 * exactly where to look for them this locator class can be used to look for the component.
 */
class ComponentLocatorBase : public ComponentAccess
//-----------------------------------------------------------------------------
{
    HLIST m_searchbase;
public:
    /// \brief Constructs a new unbound locator.
    explicit ComponentLocatorBase() : m_searchbase( INVALID_ID ) {}
    /// \brief Constructs a new bound to the specified base list locator.
    explicit ComponentLocatorBase(
        /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist ) : ComponentAccess( baselist ), m_searchbase( baselist ) {}
    /// \brief Constructs a new locator and searches the search base list.
    explicit ComponentLocatorBase(
        /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist,
        /// [in] The name or path ('/' separated) to the search base.
        const std::string& pathToSearchBase ) : ComponentAccess( baselist ), m_searchbase( INVALID_ID )
    {
        bindSearchBase( baselist, pathToSearchBase );
    }
    /// \brief Binds an access object to an internal driver object.
    /**
     * \return
     * - true if successful
     * - false otherwise. In case false is returned by this function the component could not
     * be bound, which means subsequent calls using \a access will fail or throw an exception.
     */
    bool bindComponent( /// [in,out] The access object to bind to the driver object.
        Component& access,
        /// [in] The path and/or name of the object to be located.
        const std::string& name,
        /// [in] Specifies how and what to search for. Valid flags(these flags can be combined using the '|' operator) for this parameter
        /// are:
        /// - smIgnoreLists
        /// - smIgnoreMethods
        /// - smIgnoreProperties
        int searchMode = 0,
        /// [in] The maxium depth (in lists) where to search for the component.
        /// By e.g. setting this value to 2, the current list and all
        /// its sub lists will be searched for the object, but no sublists
        /// of sub lists. -1 will search in ALL sub lists.
        int maxSearchDepth = INT_MAX ) const
    {
        access.m_hObj = findComponent( name, searchMode, maxSearchDepth );
        return access.isValid();
    }
    /// \brief Assign a new search base to the locator.
    /**
     *  This new search base will be searched starting from the base list specified.
     *  \return The unique identifier of the new search base.
     */
    HLIST bindSearchBase(
        /// [in] A unique identifier for the base list used to start searching the search base.
        HLIST baselist,
        /// [in] The name or path to the search base.
        const std::string& pathToSearchBase = "" )
    {
        if( pathToSearchBase.empty() )
        {
            m_hObj = m_searchbase = baselist;
        }
        else
        {
            TPROPHANDLING_ERROR result;
            HLIST hList;
            if( ( result = OBJ_GetHandleEx( baselist, pathToSearchBase.c_str(), &hList, smIgnoreProperties | smIgnoreMethods, 0 ) ) != PROPHANDLING_NO_ERROR )
            {
                std::string explanation = "feature list '" + pathToSearchBase + "' is not available for this device ";
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, baselist, explanation );
            }
            m_hObj = baselist;
            m_searchbase = hList;
        }
        return m_searchbase;
    }
    /// \brief Tries to locate a certain component in a hierarchy of components
    /**
     *  \return A unique identifier to the component if found or \b mvIMPACT::acquire::INVALID_ID if the component couldn't
     *  be found.
     */
    HOBJ findComponent(
        /// [in] The path and/or name of the object to be located.
        const std::string& name,
        /// [in] Specifies how and what to search for. Valid flags(these flags can be combined using the '|' operator) for this parameter
        /// are:
        /// - smIgnoreLists
        /// - smIgnoreMethods
        /// - smIgnoreProperties
        int searchMode = 0,
        /// [in] The maxium depth (in lists) where to search for the component.
        /// By e.g. setting this value to 2, the current list and all
        /// its sub lists will be searched for the object, but no sublists
        /// of sub lists. -1 will search in ALL sub lists.
        int maxSearchDepth = INT_MAX ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hObj;
        if( ( result = OBJ_GetHandleEx( m_searchbase, name.c_str(), &hObj, searchMode, maxSearchDepth ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_searchbase );
        }
        return hObj;
    }
    /// \brief Returns the unique identifier of the base list from where to start searching for a component.
    /**
     *  \return The unique identifier of the base list from where searching for a component will start currently.
     */
    HLIST searchbase_id( void ) const
    {
        return m_searchbase;
    }
};

//-----------------------------------------------------------------------------
/// \brief A class to locate components within the driver.
/**
 * Every driver will offer a set of properties, methods and component lists.
 * \b mvIMPACT::acquire::Property objects contain data such as the current gain in dB, the state of a
 * digital input, etc.. \b mvIMPACT::acquire::Method objects can be executed like a normal function and
 * \b mvIMPACT::acquire::ComponentList objects are used to group certain
 * objects together to form a logical unit.
 *
 * When it's necessary to locate one or more of these objects without knowing
 * exactly where to look for them this locator class can be used to look for the component.
 */
class ComponentLocator : public ComponentLocatorBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound locator.
    explicit ComponentLocator() : ComponentLocatorBase() {}
    /// \brief Constructs a new bound to the specified base list locator.
    explicit ComponentLocator(
        /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist ) : ComponentLocatorBase( baselist ) {}
    /// \brief Constructs a new locator and searches the search base list.
    explicit ComponentLocator(  /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist,
        /// [in] The name or path ('/' separated) to the search base.
        const std::string& pathToSearchBase ) : ComponentLocatorBase( baselist, pathToSearchBase ) {}
};

//-----------------------------------------------------------------------------
/// \brief A class to iterate over component lists.
/**
 *  This object can be used to navigate through component lists of
 *  unknown content.
 *
 *  Consider the following structure:
 *
 * \code
 *  LA
 *  |-LB
 *  |-LC
 *  |  |-PE
 *  |  |-PF
 *  |  |-PG
 *  |-PD
 * \endcode
 *
 *  Where the prefix 'L' means that this is a list, 'P' that this is a property.
 *  Assuming that we have and iterator referencing list 'C', calling \b mvIMPACT::acquire::ComponentIterator::firstChild
 *  e.g. would return a new iterator referencing object 'PE', while calling \b mvIMPACT::acquire::ComponentIterator::nextSibling
 *  would have returned a reference to 'PD' and \b mvIMPACT::acquire::ComponentIterator::parent would have
 *  returned a reference to object 'LA'.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  \b "EXAMPLE 1":
 *
 *  A new \b mvIMPACT::acquire::ComponentIterator is created with the \a ID of list 'C':
 * \code
 *  ComponentIterator it(ID_of_list_C);
 *  it = it.firstChild();    // now we are referencing 'PE'
 *  it = it.lastSibling();   // moves to 'PG'
 *  it = it.firstSibling();  // moves back to PE'
 *  it = it.nextSibling();   // moves to 'PF'
 *  it = it.firstSibling();  // moves back to PE'
 *  it = it.parent();        // we are referencing 'LC' again
 * \endcode
 *
 *  \b "EXAMPLE 2":
 *
 *  Iterate over a complete list including sub lists. This will result in a list
 *  of all lists and properties that reside in the list the iterator currently
 *  is moving through to be written to the standard output. The name of the component
 *  and every parent component will be printed into the standard output:
 *
 * \code
 *  //-----------------------------------------------------------------------------
 *  void ParseList( ComponentIterator iter, const string& path = "" )
 *  //-----------------------------------------------------------------------------
 *  {
 *    while( iter.isValid() )
 *    {
 *      if( iter.isVisible() )
 *      {
 *        if( iter.isList() )
 *        {
 *          // do some list specific stuff
 *          cout << "List     " << path << iter.name() << "/" << endl;
 *          // move down into the list and append its name to the path
 *          ParseList( iter.firstChild(), path + iter.name() + "/" );
 *        }
 *        else if( iter.isProp() )
 *        {
 *          // do property specific stuff e.g. read the value
 *          Property prop(iter);
 *          cout << "Property " << path << prop.name() << "(value(s): ";
 *          unsigned int valCount = prop.valCount();
 *          for( unsigned int i=0; i<valCount; i++ )
 *          {
 *            cout << prop.readS();
 *            if( i < valCount - 1 )
 *            {
 *              cout << ", ";
 *            }
 *          }
 *          cout << ")" << endl;
 *        }
 *      }
 *      ++iter;
 *    }
 *  }
 *
 *  //-----------------------------------------------------------------------------
 *  int main( int argc, char* argv[] )
 *  //-----------------------------------------------------------------------------
 *  {
 *    ComponentList baselist;
 *    // ....
 *    ComponentIterator it(baselist);
 *    ParseList(it);
 *    // ....
 *    return 0;
 *  }
 * \endcode
 *  \elseif DOXYGEN_NET_DOCUMENTATION
 *  \b "EXAMPLE 1":
 *
 *  A new \b mvIMPACT::acquire::ComponentIterator is created with the \a ID of list 'C':
 * \code
 *  ComponentIterator it = new ComponentIterator(ID_of_list_C);
 *  it = it.firstChild();   // now we are referencing 'PE'
 *  it = it.lastSibling();  // moves to 'PG'
 *  it = it.firstSibling(); // moves back to PE'
 *  it = it.nextSibling();  // moves to 'PF'
 *  it = it.firstSibling(); // moves back to PE'
 *  it = it.parent();       // we are referencing 'LC' again
 * \endcode
 *
 *  \b "EXAMPLE 2":
 *
 *  Iterate over a complete list including sub lists. This will result in a list
 *  of all lists and properties that reside in the list the iterator currently
 *  is moving through to be written to the standard output. The name of the component
 *  and every parent component will be printed into the standard output:
 *
 * \code
 *  static void outputPropData( Property prop )
 *  {
 *    // do property specific stuff e.g. read the value
 *    Console.Write( prop.name() + "(value(s): " );
 *    uint valCount = prop.valCount();
 *    for( uint i=0; i<valCount; i++ )
 *    {
 *      Console.Write( prop.readS() );
 *      if( i < valCount - 1 )
 *      {
 *        Console.Write( ", " );
 *      }
 *    }
 *    Console.WriteLine( ")" );
 *  }
 *
 *  static void ParseList( ComponentIterator iter, String path )
 *  {
 *    while( iter.valid )
 *    {
 *      if( iter.visible )
 *      {
 *        if( iter.list )
 *        {
 *          // do some list specific stuff
 *          Console.WriteLine( "List     " + path + iter.name() );
 *          // move down into the list and append its name to the path
 *          ParseList( iter.firstChild(), path + iter.name() + "/" );
 *        }
 *        else if( iter.prop )
 *        {
 *          Console.Write( "Property " );
 *          Console.Write( path );
 *          TComponentType type = iter.type();
 *          if( type == TComponentType.ctPropInt )
 *          {
 *            outputPropData( new PropertyI(iter.hObj()) );
 *          }
 *          else if( type == TComponentType.ctPropFloat )
 *          {
 *            outputPropData( new PropertyF(iter.hObj()) );
 *          }
 *          else if( type == TComponentType.ctPropString )
 *          {
 *            outputPropData( new PropertyS(iter.hObj()) );
 *          }
 *          else
 *          {
 *            Console.WriteLine( "This property(" + iter.name() + ") is currently unsupported under .NET" );
 *          }
 *        }
 *      }
 *      iter.nextSibling();
 *    }
 *  }
 *
 *  [STAThread]
 *  static void Main(string[] args)
 *  {
 *    try
 *    {
 *      ComponentList baselist = new ComponentList;
 *      // ...
 *      ComponentIterator it = new ComponentIterator(baselist);
 *      ParseList(it);
 *      // ...
 *    }
 *    catch(ImpactException e)
 *    {
 *      Console.WriteLine(e.Message);
 *    }
 *  }
 * \endcode
 *  \endif
 */
#ifdef DOTNET_ONLY_CODE
//-----------------------------------------------------------------------------
class ComponentIterator : public Component
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ComponentIterator object.
    explicit ComponentIterator( HOBJ hObj ) : Component( hObj ) {}
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::ComponentIterator object.
    explicit ComponentIterator() {}
    /// \brief Moves to the next sibling.
    /**
     *  \return A self reference
     */
    ComponentIterator& operator++( void ) // prefix
    {
        TPROPHANDLING_ERROR result;
        HOBJ nextSibling;
        if( ( result = OBJ_GetNextSibling( m_hObj, &nextSibling ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        m_hObj = nextSibling;
        return *this;
    }
    /// \brief Moves to the next sibling.
    /**
     *  \return A new iterator object
     */
    ComponentIterator operator++( int ) // postfix
    {
        ComponentIterator temp( *this );
        ++*this;
        return temp;
    }
    /// \brief Moves to the first sibling(the first feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::ComponentIterator object
     */
    ComponentIterator firstSibling( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetFirstSibling( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? ComponentIterator( hNew ) : ComponentIterator();
    }
    /// \brief Moves to the last sibling(the last feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::ComponentIterator object
     */
    ComponentIterator lastSibling( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetLastSibling( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? ComponentIterator( hNew ) : ComponentIterator();
    }
    /// \brief Moves to the next sibling(the next feature in the current list of features).
    /**
     *  \return A new \b mvIMPACT::acquire::ComponentIterator object
     */
    ComponentIterator nextSibling( void )
    {
        return ++*this;
    }
    /// \brief Moves to the first child of this component(moves down one level).
    /**
     *  Calling this function will only succeed, if the current \b mvIMPACT::acquire::ComponentIterator
     *  references a list.
     *
     *  \return A new \b mvIMPACT::acquire::ComponentIterator object
     */
    ComponentIterator firstChild( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetFirstChild( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? ComponentIterator( hNew ) : ComponentIterator();
    }
    /// \brief Moves to the parent of this component(moves up one level).
    /**
     *  \return A new \b mvIMPACT::acquire::ComponentIterator object
     */
    ComponentIterator parent( void ) const
    {
        TPROPHANDLING_ERROR result;
        HOBJ hNew = INVALID_ID;
        if( ( result = OBJ_GetParent( m_hObj, &hNew ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return ( hNew != INVALID_ID ) ? ComponentIterator( hNew ) : ComponentIterator();
    }
};
#else
typedef Component ComponentIterator;
#endif // #ifdef DOTNET_ONLY_CODE

//-----------------------------------------------------------------------------
/// \brief A class to provide access to component lists.
/**
 *  This class provides access to internal component lists of the driver.
 */
class ComponentList : public Component
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ComponentList object.
    explicit ComponentList(
        /// [in] A valid handle to a list object
        HLIST hList ) : Component( hList )
    {
        if( !isList() )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_NOT_A_LIST, hList );
        }
    }
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::ComponentList object.
    explicit ComponentList() {}
    /// \brief Returns The number of valid component entries in the referenced list.
    /**
     *  \return The number of valid component entries in the referenced list.
     */
    unsigned int size( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int size;
        if( ( result = OBJ_GetElementCount( m_hObj, &size ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return size;
    }
    /// \brief Returns a string containing information about what's in this list.
    /**
     *  This string might not be defined by every list but in any case this function
     *  will return a valid string. However this string might be empty.
     */
    std::string contentDescriptor( void ) const
    {
        return compGetStringParam( sqListContentDescriptor );
    }
};

//-----------------------------------------------------------------------------
/// \brief A class to call arbitrary driver functions.
/**
 *  Normally all functions needed by the user and offered by the driver will be
 *  provided as a normal function somewhere within this interface. In rare cases
 *  however it might be necessary to call a function, which hasn't been implemented
 *  in this interface. In that case this class will serve as a backdoor.
 */
class Method : public Component
//-----------------------------------------------------------------------------
{
    // do not allow assignments
    Method& operator=( const Method& );
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::Method object.
    explicit Method(
        /// [in] A valid handle to method object
        HOBJ hMeth ) : Component( hMeth )
    {
        if( !isMeth() )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_NOT_A_METHOD, hMeth );
        }
    }
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::Method object.
    explicit Method() {}
    /// \brief Calls an underlying driver function.
    /**
     * This function can be used to call any driver function which is registered
     * for the device it is called for. To call a function successfully the parameters
     * passed to the function must match the parameters expected by the function.
     *
     * All parameters are passed as a single string, which is parsed with respect to the
     * given delimiter characters internally.
     *
     * To find out what kind of parameters are expected by the function use
     * the function \b mvIMPACT::acquire::Method::paramList().
     *
     * floating point values can be passed either with a '.' or a ',' acting as the
     * decimal point.
     *
     * 'empty' strings can be passed as a single underline('_').
     *
     * \warning
     * The characters '.', ',' and '_' can't be used as delimiters.
     *
     * \if DOXYGEN_CPP_DOCUMENTATION
     *
     * \code
     * // call a function expecting a string and a float
     * // value parameters separated by spaces
     * meth.call( "stringParam 3,14" );
     * // call of a function expecting 2 integers an a string
     * // where an empty string shall be passed
     * // parameters are separated by '%'
     * meth.call( "1000%666%_", "%" );
     * \endcode
     * \endif
     * \return An integer value.
     */
    int call(
        /// [in] The parameters to be passed to the function as a single string
        const std::string& params,
        /// [in] A string containing valid delimiter characters for the parameter string
        const std::string& delimiters = " " ) const
    {
        TPROPHANDLING_ERROR result;
        int retval;
        if( ( result = OBJ_Execute( m_hObj, params.c_str(), delimiters.c_str(), &retval ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return retval;
    }
    /// \brief Calls an underlying driver function expecting no parameters.
    /**
     * This function can be used to call any driver function which is registered
     * for the device it is called for that does \b NOT expect any parameters
     *
     * To find out what kind of parameters are expected by the function use
     * the function \b "mvIMPACT::acquire::Method::paramList() const".
     *
     * \return An integer value.
     */
    int call( void ) const
    {
        TPROPHANDLING_ERROR result;
        int retval;
        if( ( result = OBJ_Execute( m_hObj, 0, 0, &retval ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return retval;
    }
    /// \brief Returns the parameter list of the methods as a string.
    /**
     *  This function returns a string containing one character for each parameter this \b mvIMPACT::acquire::Method
     *  object expects and one for the return type of the function call.
     *
     *  The first character is the return type of the function all others are parameters. void functions
     *  don't specify parameters.
     *
     *  The characters have the following meaning:
     *  - i specifies a 32-bit integer value
     *  - I specifies a 64-bit integer value
     *  - s specifies a pointer to a C-string
     *  - f specifies a double precision float value
     *  - p specifies a pointer value
     *  - v specifies a void return value
     *
     *  \b EXAMPLES:
     *
     *  - 'v': This is a function returning nothing (void). It expects no parameters.
     *  - 'viis': This is a function returning nothing (void). It expects 2 integer values
     *  and one pointer to a C-string.
     *  - 'if': This function returns an integer value and expects a float value.
     *  \return The parameter list of the method as a string.
     */
    std::string paramList( void ) const
    {
        return compGetStringParam( sqMethParamString );
    }
};

//-----------------------------------------------------------------------------
/// \brief Defines valid limits which can be queried for a \b mvIMPACT::acquire::Property object.
enum TPropertyLimits
//-----------------------------------------------------------------------------
{
    /// \brief Set/Get the maximum value for this \b mvIMPACT::acquire::Property.
    /**
     *  Pass this as the index to set or get the maximum number of values this property
     *  can store with a call to the \b mvIMPACT::acquire::Property object's corresponding
     *  \b read or \b write method.
     */
    plMaxValue = PROP_MAX_VAL,
    /// \brief Set/Get the minimum value for this \b mvIMPACT::acquire::Property.
    /**
     *  Pass this as the index to set or get the maximum number of values this property
     *  can store with a call to the \b mvIMPACT::acquire::Property object's corresponding
     *  \b read or \b write method.
     */
    plMinValue = PROP_MIN_VAL,
    /// \brief Set/Get the step width value for this \b mvIMPACT::acquire::Property.
    /**
     *  Pass this as the index to set or get the maximum number of values this property
     *  can store with a call to the \b mvIMPACT::acquire::Property object's corresponding
     *  \b read or \b write method.
     */
    plStepWidth = PROP_STEP_WIDTH
};

//-----------------------------------------------------------------------------
/// \brief A base class for properties.
/**
 *  A property can be used to represent certain values like e.g. the input channel
 *  of a device. Depending on the way the property has been created it is either
 *  possible to read and write data to it or ( when the \b mvIMPACT::acquire::cfWriteAccess flag is \b NOT set )
 *  just to read the data. In rare cases it might also be possible that the user is
 *  is not allowed to read the data of a property. To find out what you are allowed
 *  to do with a property or any other component the function \b mvIMPACT::acquire::Component::flags()
 *  can be called.
 *
 *  A property can contain either a single value or an array of values of the same
 *  type (e.g. 4 integer values could be used to represent a property call 'Rectangle'.
 *  It can even (if the \b mvIMPACT::acquire::cfFixedSize flag is not set) contain
 *  a different number of values all the time its data is queried. E.g. for a property
 *  'searchresults' each time some function is called which writes its results to this
 *  property the property could contain a different number of integer values afterwards.
 *
 *  Every properties value can be read and written either as string or by it actual value type.
 *  So you can either assign the value of an integer property by using the standard 'write' function
 *  that accepts the value to be the type of the property or the function \b mvIMPACT::acquire::Property::writeS
 *  can be used to set the property via a string containing the new value.
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  PropertyI prop(hObj);
 *  prop.writeS("5"); // string assignment
 *  prop.write(5); // integer assignment
 * \endcode
 *  \endif
 *
 *  In addition to that \e float and \e int Properties might define a translation dictionary. This cannot be
 *  done by the user, but the user can work with the dictionary afterwards. A translation
 *  dictionary is a table which maps strings to certain values (e.g. integers).
 *
 *  The translation dictionary serves two purposes: Once it has been defined this property can only be
 *  assigned values ( which can either be passed as a string or as the actual value )
 *  which are contained in the translation table. Thus this makes it very easy to restrict
 *  a certain property to a fixed number of values, which can be assigned to it.
 *  Properties that have a translation dictionary will typically be defined by declaring
 *  a template instance with the type of the enumeration.
 *
 *  The second benefit of a translation dictionary is that certain values can be assigned
 *  a meaningful description via its translation.
 *  When a translation table has been defined either the string translation can be used to
 *  assign its value or its actual data type. A float property e.g. might define a translation
 *  table like this: 'Auto -> 0', 'auto -> 1', 'OnLowLevel -> 2'. Now to set this property
 *  to use the 'on low level' trigger mode this can be done by calling one of the 'write' functions.
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  PropertyICameraTriggerMode& prop = getPropertyFromSomewhere();
 *  prop.writeS("OnLowLevel"); // set value via the translation string
 *  prop.write(ctmOnLowLevel); // set value via enum type
 * \endcode
 *  \endif
 *
 *  This can also be extremely useful to populate combo boxes for GUI applications.
 *
 *  Float and int properties can also define certain constants like a max. value or
 *  a min. value. If a property does define such values it can't be assigned values
 *  which do not lie within the defined range. To find out if constants are defined
 *  The user can call the functions \b mvIMPACT::acquire::Property::hasMaxValue,
 *  \b mvIMPACT::acquire::Property::hasMinValue and \b mvIMPACT::acquire::Property::hasStepWidth.
 *  To query to actual value of the max, min or step width the functions
 *  \b getMaxValue, \b getMinValue and \b getStepWidth of the classes
 *  \b mvIMPACT::acquire::EnumPropertyF, \b mvIMPACT::acquire::EnumPropertyI and
 *  \b mvIMPACT::acquire::EnumPropertyI64 can be called.
 *
 *  When reading the value of a property as a string the user can define a format string
 *  telling the property module how to format the returned string. This works pretty much
 *  like when working e.g. with the \e printf function.
 */
class Property : public Component
//-----------------------------------------------------------------------------
{
    // do not allow assignments! This makes sure the user can't assign e.g. the
    // gain property to the property for controlling the expose time
    Property& operator=( const Property& );
    bool isConstDefinedInternal( TPropertyLimits constant ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int isDefined;
        if( ( result = OBJ_IsConstantDefined( m_hObj, constant, &isDefined ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return isDefined != 0;
    }
protected:
    //-----------------------------------------------------------------------------
    int valuesToRead( int start, int end ) const
    //-----------------------------------------------------------------------------
    {
        if( start < 0 )
        {
            // when reading a set of values, these can't include limits or any other value associated with
            // a negative index parameter to avoid confusion cause by END_OF_LIST
            // having the same value as PROP_MAX_VAL.
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_INPUT_PARAMETER, m_hObj );
        }
        return ( end == END_OF_LIST ) ? ( valCount() - start ) : ( end - start + 1 );
    }
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::Property object.
    /**
     *  Properties constructed this way can't be used for anything unless they
     *  are bound to a internal driver property with a \b mvIMPACT::acquire::ComponentLocator object.
     */
    explicit Property( void ) {}
    /// \brief Constructs a new \b mvIMPACT::acquire::Property object.
    /**
     *  Properties successfully constructed this way can be worked with directly.
     *  If \a hObj does not reference an internal driver property a \b mvIMPACT::acquire::ENotAProperty
     *  exception will be thrown.
     */
    explicit Property(
        /// [in] A valid handle to a property
        HOBJ hProp ) : Component( hProp )
    {
        if( !isProp() )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_NOT_A_PROPERTY, hProp );
        }
    }
    /// \brief Checks if this enumerated property allows the combination of enum values.
    /**
     *  If this function returns true, the enum constants can be 'ored' together. This works
     *  for the enum constants as well as for the string representations returned in the
     *  properties translation dictionary.
     *
     *  if a property e.g. defines a dictionary (('1', "one"), ('2', "two")) the following
     *  write operations will be valid:
     *
     * \code
     *  prop = getPropFromSomewhere()
     *  prop.write( 1 | 2 )
     *  prop.write( "one | two")
     * \endcode
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If this function returns true code like this will be valid:
     *
     * \code
     *  enum TEnum
     *  {
     *    eA,
     *    eB
     *  };
     *
     *  typedef EnumPropertyI<TEnum> PropertyIEnum;
     *  PropertyIEnum p = getPropFromSomewhere();
     *  p.write( TEnum(eA | eB) );
     * \endcode
     *  \endif
     *  \return
     *  - true if this property allows the combination of enum values.
     *  - false otherwise.
     */
    bool allowsValueCombinations( void ) const
    {
        return ( flags() & cfAllowValueCombinations ) != 0;
    }
    /// \brief Returns the size of the properties translation dictionary.
    /**
     *  If the property does define a translation dictionary this function returns
     *  the number of elements contained in this dictionary.
     *  \return
     *  - The size of the properties translation dictionary.
     *  - 0 If this property does not define a translation dictionary.
     */
    unsigned int dictSize( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int size = 0;
        if( ( result = OBJ_GetDictSize( m_hObj, &size ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return size;
    }
    /// \brief Returns whether this property defines a translation dictionary or not.
    /**
     *  \return
     *  - true if this property defines a translation table
     *  - false otherwise
     */
    bool hasDict( void ) const
    {
        return ( dictSize() > 0 );
    }
    /// \brief Checks if a certain constant is defined for this property(\b deprecated).
    /**
     *  \deprecated
     *  This function has been declared deprecated and will be removed in future versions of this interface.
     *  Use \b mvIMPACT::acquire::Property::hasMaxValue(), \b mvIMPACT::acquire::Property::hasMinValue()
     *  and \b mvIMPACT::acquire::Property::hasStepWidth() instead and see the corresponding 'Porting existing code'
     *  section in the documentation.
     *
     *  Valid values for \a constant are defined by the enum \b mvIMPACT::acquire::TPropertyLimits.
     *  \return
     *  - true if this property defines this constant
     *  - false otherwise
     */
    MVIMPACT_DEPRECATED_CPP( bool isConstDefined(
                                 /// [in] The constant whose existance is in question
                                 TPropertyLimits constant ) const );
    /// \brief Checks if a maximum value is defined for this property
    /**
     *  \return
     *  - true if this property defines a maximum value
     *  - false otherwise
     */
    bool hasMaxValue( void ) const
    {
        return isConstDefinedInternal( plMaxValue );
    }
    /// \brief Checks if a minimum value is defined for this property
    /**
     *  \return
     *  - true if this property defines a minimum value
     *  - false otherwise
     */
    bool hasMinValue( void ) const
    {
        return isConstDefinedInternal( plMinValue );
    }
    /// \brief Checks if a step width is defined for this property
    /**
     *  \return
     *  - true if this property defines a step width
     *  - false otherwise
     */
    bool hasStepWidth( void ) const
    {
        return isConstDefinedInternal( plStepWidth );
    }
    /// \brief Reads data from this property as a string.
    /**
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \return A string containing the data questioned.
     */
    std::string readS(
        /// [in] The index of the desired value(only necessary for properties containing more than one value)
        int index = 0,
        /// [in] The format string telling the function how to format the result. If left empty the property uses its standard way of converting its data into a string
        const std::string& format = "" ) const
    {
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pBuf = new char[bufSize];
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        while( ( result = OBJ_GetSFormattedEx( m_hObj, pBuf, &bufSize, ( ( format != "" ) ? format.c_str() : 0 ), index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            delete [] pBuf;
            pBuf = new char[bufSize];
        }
        std::string value( pBuf );
        delete [] pBuf;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return value;
    }
    /// \brief Reads data from this property as a string.
    /**
     *  This function can be used to query a set of values if this property stores more
     *  than a single value which might be useful for GUI applications.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  PropertyI p = getPropFromSomewhere();
     *  p.write( 1 );
     *  p.write( 2, 1 );
     *  p.write( 666, 2 );
     *  std::string s = p.readSArray( "%d", "&&:", 1 );
     *  // now s should contain '2&&:666'
     *  s = p.readSArray( "%4d", " ", 0, 1 );
     *  // now s should contain '   1,   2'
     * \endcode
     *  \endif
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \return A string containing the data questioned.
     */
    std::string readSArray(
        /// [in] The format string telling the function how to format the result. If left empty the property uses its standard way of converting its data into a string
        const std::string& format = "",
        /// [in] This string is used to separate the individual values from one another. If left empty, a single blank will separate
        /// the data.
        const std::string& delimiter = "",
        /// [in] The index of the first of the desired values
        int startIndex = 0,
        /// [in] The index of the last of the desired values. If \b INT_MAX is passed, every
        /// from \a startIndex to the last value stored by the property will be returned.
        int endIndex = INT_MAX,
        /// [in] Specifies the mode this function operates in. Currently only the LSB affects the behaviour of this function.
        /// When the LSB is set, the translation dictionary (if defined) will be ignored for this call.
        int mode = 0 ) const
    {
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pBuf = new char[bufSize];
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        while( ( result = OBJ_GetSArrayFormattedEx( m_hObj,
                          pBuf,
                          &bufSize,
                          ( format != "" ) ? format.c_str() : 0,
                          ( delimiter != "" ) ? delimiter.c_str() : 0,
                          startIndex,
                          endIndex,
                          mode ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            delete [] pBuf;
            pBuf = new char[bufSize];
        }
        std::string value( pBuf );
        delete [] pBuf;
        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return value;
    }
    /// \brief Removes a certain value from the property's data.
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    void
#else
    /**
     *  \return A const reference to the calling property.
     */
    const Property&
#endif
    removeValue(
        /// [in] The index of the value to be removed
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_RemoveVal( m_hObj, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Resizes the property's data array.
    /**
     *  This function resizes the internal data array of this property. The size of
     *  this array represents the number of values, which can be stored within the
     *  property. This function will only succeed, if the \b mvIMPACT::acquire::cfFixedSize
     *  is \b NOT set for this property and the user has \b "write rights" for this property.
     *  Otherwise an exception will be thrown. Whenever the user successfully writes an
     *  array of values to a property and this array contains more elements than the current
     *  internal data array can accommodate at the desired offset the internal data array
     *  will be increased automatically.
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \note In order to be allowed to modify the number of values a property
     *  can store, the \b mvIMPACT::acquire::cfFixedSize flag
     *  must \b NOT be set.
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable
     */
    void
#else
    /// \brief Resizes the property's data array.
    /**
     *  This function resizes the internal data array of this property. The size of
     *  this array represents the number of values, which can be stored within the
     *  property. This function will only succeed, if the \b mvIMPACT::acquire::cfFixedSize
     *  is \b NOT set for this property and the user has \b "write rights" for this property.
     *  Otherwise an exception will be thrown. Whenever the user successfully writes an
     *  array of values to a property and this array contains more elements than the current
     *  internal data array can accommodate at the desired offset the internal data array
     *  will be increased automatically.
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \note In order to be allowed to modify the number of values a property
     *  can store, the \b mvIMPACT::acquire::cfFixedSize flag
     *  must \b NOT be set.
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable
     *
     *  \return A const reference to the calling property.
     */
    const Property&
#endif
    resizeValArray(
        /// [in] The new number of values this property shall be allowed to store
        unsigned int newSize ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetValCount( m_hObj, newSize ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
    /// \brief Returns the internal format string this property uses to convert data to strings
    /**
     *  \return A string containing the format string internally used by the property
     *  whenever the user reads a value as a string without specifying a format string and
     *  the property is not a string type one.
     */
    std::string stringFormatString( void ) const
    {
        return compGetStringParam( sqPropFormatString );
    }
    /// \brief Returns the current number of values managed by this property.
    /**
     *  For the majority of properties this function will return '1', but as properties
     *  might manage more than a single value, this value might be interesting from time
     *  to time.
     *  \return the current number of values managed by this property.
     */
    unsigned int valCount( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int valCount;
        if( ( result = OBJ_GetValCount( m_hObj, &valCount ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return valCount;
    }
    /// \brief Returns maximum number of values that can be managed by this property.
    /**
     *  For the majority of properties this function will return '1', but as properties
     *  might manage more than a single value, this value might be interesting from time
     *  to time.
     *  \return the maximum number of values that can be managed by this property.
     */
    unsigned int maxValCount( void ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int maxValCount;
        if( ( result = OBJ_GetMaxValCount( m_hObj, &maxValCount ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return maxValCount;
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Assigns a new value to this property
    /**
     *  The user caller have \b "write rights" for this property in order to be able to
     *  modify its value. Also if \a index is greater than the current internal data array
     *  size of this property the user must be allowed to change to size of the properties
     *  internal data array (the \b mvIMPACT::acquire::cfFixedSize flag must \b NOT be set).
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable, \n\b mvIMPACT::acquire::Component::flags, \n
     *  \b mvIMPACT::acquire::Property::writeS
     */
    void
#else
    /// \brief Assigns a new value to this property
    /**
     *  The user caller have \b "write rights" for this property in order to be able to
     *  modify its value. Also if \a index is greater than the current internal data array
     *  size of this property the user must be allowed to change to size of the properties
     *  internal data array (the \b mvIMPACT::acquire::cfFixedSize flag must \b NOT be set).
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable, \n\b mvIMPACT::acquire::Component::flags, \n
     *  \b mvIMPACT::acquire::Property::writeS
     *
     *  \return A const reference to the calling property.
     */
    const Property&
#endif
    writeS(
        /// [in] The new value for this property at the given \a index
        const std::string& value,
        /// [in] The index of the value to modify
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetS( m_hObj, value.c_str(), index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj, value );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Assigns new values to this property
    /**
     *  The user must have \b "write rights" for this property in order to be able to
     *  modify its values. Also if \a index is greater than the current internal data array
     *  size of this property the user must be allowed to change to size of the properties
     *  internal data array (the \b mvIMPACT::acquire::cfFixedSize flag must \b NOT be set). This
     *  function behaves exactly like \b mvIMPACT::acquire::Property::writeS
     *  except that is can be used to assign more than one value at the same time. The parameter
     *  \a index here serves as an offset. If for example a property holds 3 values 'one', 'two'
     *  and 'three' before this call and the function is then called with an array containing the
     *  string 'orange' and 'blue' and \a index = 2 then after a successful call to this function
     *  the property will hold the data 'one', 'two', 'orange' and 'blue'.
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable, \n \b mvIMPACT::acquire::Component::flags, \n
     *  \b mvIMPACT::acquire::Property::writeS
     */
    void
#else
    /// \brief Assigns new values to this property
    /**
     *  The user must have \b "write rights" for this property in order to be able to
     *  modify its values. Also if \a index is greater than the current internal data array
     *  size of this property the user must be allowed to change to size of the properties
     *  internal data array (the \b mvIMPACT::acquire::cfFixedSize flag must \b NOT be set). This
     *  function behaves exactly like \b mvIMPACT::acquire::Property::writeS
     *  except that is can be used to assign more than one value at the same time. The parameter
     *  \a index here serves as an offset. If for example a property holds 3 values 'one', 'two'
     *  and 'three' before this call and the function is then called with an array containing the
     *  string 'orange' and 'blue' and \a index = 2 then after a successful call to this function
     *  the property will hold the data 'one', 'two', 'orange' and 'blue'.
     *
     *  \note If the caller does not have the needed rights this function might throw an
     *  exception of the type \b mvIMPACT::acquire::ImpactAcquireException.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Component::isWriteable, \n \b mvIMPACT::acquire::Component::flags, \n
     *  \b mvIMPACT::acquire::Property::writeS
     *
     *  \return A const reference to the calling property.
     */
    const Property&
#endif
    writeS(
        /// [in] A constant reference to an array containing the strings to be assigned to the property
        const std::vector<std::string>& sequence,
        /// [in] The offset from where to start to assign the values
        int index = 0 ) const
    {
        unsigned int vSize = static_cast<unsigned int>( sequence.size() );
        for( unsigned int i = 0; i < vSize; i++ )
        {
            writeS( sequence[i], index + i );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#   if !defined(WRAP_PYTHON) // To date, no customer uses Python, so we don't need backward compatibility here.
//-----------------------------------------------------------------------------
inline bool Property::isConstDefined( TPropertyLimits constant ) const
//-----------------------------------------------------------------------------
{
    return isConstDefinedInternal( constant );
}
#   endif // #if !defined(WRAP_PYTHON)
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

//-----------------------------------------------------------------------------
/// \brief A template class to represent float properties and enumerated float properties
/**
 *  The template parameter specifies the enum to use as the value type for this
 *  enumerated property. The class itself however is restricted for float values.
 *  To use a normal float (not enumerated) properties use the \b mvIMPACT::acquire::PropertyF type.
 */
template<class ZYX>
class EnumPropertyF : public Property
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::EnumPropertyF object.
    explicit EnumPropertyF() {}
    /// \brief Constructs a new \b mvIMPACT::acquire::EnumPropertyF object.
    explicit EnumPropertyF(
        /// [in] A valid handle to a float property
        HOBJ hProp ) : Property( hProp )
    {
        if( type() != ctPropFloat )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_PROP_VALUE_TYPE, hProp );
        }
    }
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    typedef ZYX value_type;
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
#ifndef WRAP_DOTNET
    /// \brief This function queries the property's translation table
    /**
     *  If this property defines a translation table the strings and their corresponding
     *  translation values will be written into \a sequence. If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b EnumPropertyF::getTranslationDictString and
     *  \b EnumPropertyF::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyF& getTranslationDict(
        /// [out] A reference to a container which will receive the data from the properties translation dictionary.
        std::vector<std::pair<std::string, ZYX> >& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        double* pVal = new double[size];
        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        size_t i = 0;

        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetFDictEntries( m_hObj, ppBuf, bufSize, pVal, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::pair<std::string, ZYX>( ppBuf[i], static_cast<ZYX>( pVal[i] ) );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;
        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief This function reads a single translation table string entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the string representation of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     */
    std::string getTranslationDictString(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pTranslationString = new char[bufSize];
        while( ( result = OBJ_GetFDictEntry( m_hObj, pTranslationString, bufSize, 0, index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            delete [] pTranslationString;
            pTranslationString = new char[bufSize];
        }

        std::string translationString( pTranslationString );
        delete [] pTranslationString;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return translationString;
    }
    /// \brief This function reads a single translation table value entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the value of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     *  \sa \b mvIMPACT::acquire::EnumPropertyF::getTranslationDictValues to query all valid
     *  values efficiently in a single call.
     */
    ZYX getTranslationDictValue(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        double value;
        if( ( result = OBJ_GetFDictEntry( m_hObj, 0, 0, &value, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( result );
    }
#ifndef WRAP_DOTNET
    /// \brief This function queries a list of valid strings for this property
    /**
     *  If this property defines a translation table all valid strings will be written into \a sequence.
     *  If the property does \b NOT define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyF::getTranslationDictString multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyF& getTranslationDictStrings(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<std::string>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        size_t i = 0;

        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetFDictEntries( m_hObj, ppBuf, bufSize, 0, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::string( ppBuf[i] );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }

    /// \brief This function queries a list of valid values for this property
    /**
     *  If this property defines a translation table all valid values will be written into \a sequence.
     *  If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyF::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyF& getTranslationDictValues(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        const unsigned int size = dictSize();
        if( size == 0 )
        {
            sequence.clear();
            return *this;
        }

        double* pVal = new double[size];

        if( ( result = OBJ_GetFDictEntries( m_hObj, 0, 0, pVal, size ) ) == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = static_cast<ZYX>( pVal[i] );
            }
        }

        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief Reads a value from a property.
    /**
     *  This function queries a single value stored under index \a index in the property.
     *  \return The value stored at \a index within the property.
     */
    ZYX read(
        /// [in] The index of the value to read from the property.
        int index = 0 ) const
    {
        double val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetF( m_hObj, &val, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( val );
    }
    /// \brief Reads the maximum value from a property.
    /**
     *  \note
     *  To find out if the property defines a maximum value \b mvIMPACT::acquire::Property::hasMaxValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a maximum value calling this function will raise
     *  an exception.
     */
    ZYX getMaxValue( void ) const
    {
        return read( plMaxValue );
    }
    /// \brief Reads the minimum value from a property.
    /**
     *  \note
     *  To find out if the property defines a minimum value \b mvIMPACT::acquire::Property::hasMinValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a minimum value calling this function will raise
     *  an exception.
     */
    ZYX getMinValue( void ) const
    {
        return read( plMinValue );
    }
    /// \brief Reads the step width from a property.
    /**
     *  \note
     *  To find out if the property defines a step width \b mvIMPACT::acquire::Property::hasStepWidth
     *  should be called first.
     *
     *  \note
     *  If the property does not define a step width calling this function will raise
     *  an exception.
     */
    ZYX getStepWidth( void ) const
    {
        return read( plStepWidth );
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        unsigned int valCnt = valuesToRead( start, end );
        sequence.resize( valCnt );
        for( unsigned int i = 0; i < valCnt; i++ )
        {
            sequence[i] = read( i + start );
        }
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to get all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be read one after the other
        bool boAtomic,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        if( boAtomic )
        {
            unsigned int valCnt = valuesToRead( start, end );
            sequence.resize( valCnt );
            double* pValues = new double[valCnt];
            TPROPHANDLING_ERROR result = OBJ_GetFArray( m_hObj, pValues, valCnt, start );
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                sequence[i] = static_cast<ZYX>( pValues[i] );
            }
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
        }
        else
        {
            read( sequence, start, end );
        }
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     */
    void
#else
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyF&
#endif
    write(
        /// [in] The value to write to the property.
        ZYX value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetF( m_hObj, static_cast<double>( value ), index ) ) != PROPHANDLING_NO_ERROR )
        {
            std::ostringstream oss;
            oss << value;
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj, oss.str() );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyF&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        unsigned int vSize = static_cast<unsigned int>( sequence.size() );
        for( unsigned int i = 0; i < vSize; i++ )
        {
            write( sequence[i], index + i );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyF&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to set all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be set one after the other
        bool boAtomic,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        if( boAtomic )
        {
            const unsigned int valCnt = static_cast<unsigned int>( sequence.size() );
            double* pValues = new double[valCnt];
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                pValues[i] = sequence[i];
            }
            TPROPHANDLING_ERROR result = OBJ_SetFArray( m_hObj, pValues, valCnt, index );
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
#ifndef DOTNET_ONLY_CODE
            return *this;
#endif
        }
        else
        {
            return write( sequence, index );
        }
    }
};

/// \brief A type for floating point properties.
/**
 *  Provided for convenience only. This type represents a standard float property type.
 */
typedef EnumPropertyF<double> PropertyF;
PYTHON_ONLY( ENUM_PROPERTY( PropertyF, EnumPropertyF, double ) )

//-----------------------------------------------------------------------------
/// \brief A template class to represent 32 bit integer properties and 32 bit enumerated integer properties
/**
 *  The template parameter specifies the enum to use as the value type for this
 *  enumerated property. The class itself however is restricted for integer values.
 *  To use a normal int (not enumerated) properties use the \b mvIMPACT::acquire::PropertyI type.
 */
template<class ZYX>
class EnumPropertyI : public Property
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::EnumPropertyI object.
    explicit EnumPropertyI() {}
    /// \brief Constructs a new \b mvIMPACT::acquire::EnumPropertyI object.
    explicit EnumPropertyI(
        /// [in] A valid handle to an integer property
        HOBJ hProp ) : Property( hProp )
    {
        const TComponentType compType = type();
        if( ( compType != ctPropInt ) && ( compType != ctPropInt64 ) )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_PROP_VALUE_TYPE, hProp );
        }
    }
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    typedef ZYX value_type;
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
#ifndef WRAP_DOTNET
    /// \brief This function queries the property's translation table
    /**
     *  If this property defines a translation table the strings and their corresponding
     *  translation values will be written into \a sequence. If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictString and
     *  \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI& getTranslationDict(
        /// [out] A reference to a container which will receive the data from the properties translation dictionary.
        std::vector<std::pair<std::string, ZYX> >& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        int* pVal = new int[size];
        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT, i;
        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetIDictEntries( m_hObj, ppBuf, bufSize, pVal, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::pair<std::string, ZYX>( ppBuf[i], static_cast<ZYX>( pVal[i] ) );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;
        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief This function reads a single translation table string entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the string representation of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     */
    std::string getTranslationDictString(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pTranslationString = new char[bufSize];
        while( ( result = OBJ_GetIDictEntry( m_hObj, pTranslationString, bufSize, 0, index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            delete [] pTranslationString;
            pTranslationString = new char[bufSize];
        }

        std::string translationString( pTranslationString );
        delete [] pTranslationString;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return translationString;
    }
    /// \brief This function reads a single translation table value entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the value of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     *  \sa \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictValues to query all valid
     *  values efficiently in a single call.
     */
    ZYX getTranslationDictValue(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        int value;
        if( ( result = OBJ_GetIDictEntry( m_hObj, 0, 0, &value, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( value );
    }
#ifndef WRAP_DOTNET
    /// \brief This function queries a list of valid strings for this property
    /**
     *  If this property defines a translation table all valid strings will be written into \a sequence.
     *  If the property does \b NOT define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictString multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI& getTranslationDictStrings(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<std::string>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        size_t i = 0;

        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetIDictEntries( m_hObj, ppBuf, bufSize, 0, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::string( ppBuf[i] );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }

    /// \brief This function queries a list of valid values for this property
    /**
     *  If this property defines a translation table all valid values will be written into \a sequence.
     *  If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI& getTranslationDictValues(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        const unsigned int size = dictSize();
        if( size == 0 )
        {
            sequence.clear();
            return *this;
        }

        int* pVal = new int[size];

        if( ( result = OBJ_GetIDictEntries( m_hObj, 0, 0, pVal, size ) ) == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = static_cast<ZYX>( pVal[i] );
            }
        }

        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief Reads a value from a property.
    /**
     *  This function queries a single value stored under index \a index in the property.
     *  \return The value stored at \a index within the property.
     */
    ZYX read(
        /// [in] The index of the value to read from the property.
        int index = 0 ) const
    {
        int val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetI( m_hObj, &val, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( val );
    }
    /// \brief Reads the maximum value from a property.
    /**
     *  \note
     *  To find out if the property defines a maximum value \b mvIMPACT::acquire::Property::hasMaxValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a maximum value calling this function will raise
     *  an exception.
     */
    ZYX getMaxValue( void ) const
    {
        return read( plMaxValue );
    }
    /// \brief Reads the minimum value from a property.
    /**
     *  \note
     *  To find out if the property defines a minimum value \b mvIMPACT::acquire::Property::hasMinValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a minimum value calling this function will raise
     *  an exception.
     */
    ZYX getMinValue( void ) const
    {
        return read( plMinValue );
    }
    /// \brief Reads the step width from a property.
    /**
     *  \note
     *  To find out if the property defines a step width \b mvIMPACT::acquire::Property::hasStepWidth
     *  should be called first.
     *
     *  \note
     *  If the property does not define a step width calling this function will raise
     *  an exception.
     */
    ZYX getStepWidth( void ) const
    {
        return read( plStepWidth );
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        unsigned int valCnt = valuesToRead( start, end );
        sequence.resize( valCnt );
        for( unsigned int i = 0; i < valCnt; i++ )
        {
            sequence[i] = read( i + start );
        }
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to get all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be read one after the other
        bool boAtomic,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        if( boAtomic )
        {
            unsigned int valCnt = valuesToRead( start, end );
            sequence.resize( valCnt );
            int* pValues = new int[valCnt];
            TPROPHANDLING_ERROR result = OBJ_GetIArray( m_hObj, pValues, valCnt, start );
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                sequence[i] = static_cast<ZYX>( pValues[i] );
            }
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
        }
        else
        {
            read( sequence, start, end );
        }
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     */
    void
#else
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI&
#endif
    write(
        /// [in] The value to write to the property.
        ZYX value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetI( m_hObj, static_cast<int>( value ), index ) ) != PROPHANDLING_NO_ERROR )
        {
            std::ostringstream oss;
            oss << value;
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj, oss.str() );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        int vSize = static_cast<int>( sequence.size() );
        for( int i = 0; i < vSize; i++ )
        {
            write( sequence[i], index + i );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to set all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be set one after the other
        bool boAtomic,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        if( boAtomic )
        {
            const unsigned int valCnt = static_cast<unsigned int>( sequence.size() );
            int* pValues = new int[valCnt];
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                pValues[i] = sequence[i];
            }
            TPROPHANDLING_ERROR result = OBJ_SetIArray( m_hObj, pValues, valCnt, index );
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
#ifndef DOTNET_ONLY_CODE
            return *this;
#endif
        }
        else
        {
            return write( sequence, index );
        }
    }
};

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAcquisitionMode
typedef EnumPropertyI<TAcquisitionMode> PropertyIAcquisitionMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAcquisitionMode, EnumPropertyI, mvIMPACT::acquire::TAcquisitionMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAcquisitionStartStopBehaviour
typedef EnumPropertyI<TAcquisitionStartStopBehaviour> PropertyIAcquisitionStartStopBehaviour;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAcquisitionStartStopBehaviour, EnumPropertyI, mvIMPACT::acquire::TAcquisitionStartStopBehaviour ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAoiMode
typedef EnumPropertyI<TAoiMode> PropertyIAoiMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAoiMode, EnumPropertyI, mvIMPACT::acquire::TAoiMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBayerConversionMode
typedef EnumPropertyI<TBayerConversionMode> PropertyIBayerConversionMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBayerConversionMode, EnumPropertyI, mvIMPACT::acquire::TBayerConversionMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBayerMosaicParity
typedef EnumPropertyI<TBayerMosaicParity> PropertyIBayerMosaicParity;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBayerMosaicParity, EnumPropertyI, mvIMPACT::acquire::TBayerMosaicParity ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBayerWhiteBalanceResult
typedef EnumPropertyI<TBayerWhiteBalanceResult> PropertyIBayerWhiteBalanceResult;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBayerWhiteBalanceResult, EnumPropertyI, mvIMPACT::acquire::TBayerWhiteBalanceResult ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBoolean
typedef EnumPropertyI<TBoolean> PropertyIBoolean;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBoolean, EnumPropertyI, mvIMPACT::acquire::TBoolean ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraOutput
typedef EnumPropertyI<TCameraOutput> PropertyICameraOutput;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraOutput, EnumPropertyI, mvIMPACT::acquire::TCameraOutput ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TChannelSplitMode
typedef EnumPropertyI<TChannelSplitMode> PropertyIChannelSplitMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIChannelSplitMode, EnumPropertyI, mvIMPACT::acquire::TChannelSplitMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TColorTwistInputCorrectionMatrixMode
typedef EnumPropertyI<TColorTwistInputCorrectionMatrixMode> PropertyIColorTwistInputCorrectionMatrixMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIColorTwistInputCorrectionMatrixMode, EnumPropertyI, mvIMPACT::acquire::TColorTwistInputCorrectionMatrixMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TColorTwistOutputCorrectionMatrixMode
typedef EnumPropertyI<TColorTwistOutputCorrectionMatrixMode> PropertyIColorTwistOutputCorrectionMatrixMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIColorTwistOutputCorrectionMatrixMode, EnumPropertyI, mvIMPACT::acquire::TColorTwistOutputCorrectionMatrixMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TColorProcessingMode
typedef EnumPropertyI<TColorProcessingMode> PropertyIColorProcessingMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIColorProcessingMode, EnumPropertyI, mvIMPACT::acquire::TColorProcessingMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDarkCurrentFilterMode
typedef EnumPropertyI<TDarkCurrentFilterMode> PropertyIDarkCurrentFilterMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDarkCurrentFilterMode, EnumPropertyI, mvIMPACT::acquire::TDarkCurrentFilterMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDefectivePixelsFilterMode
typedef EnumPropertyI<TDefectivePixelsFilterMode> PropertyIDefectivePixelsFilterMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDefectivePixelsFilterMode, EnumPropertyI, mvIMPACT::acquire::TDefectivePixelsFilterMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceAccessMode
typedef EnumPropertyI<TDeviceAccessMode> PropertyIDeviceAccessMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceAccessMode, EnumPropertyI, mvIMPACT::acquire::TDeviceAccessMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceCapability
typedef EnumPropertyI<TDeviceCapability> PropertyIDeviceCapability;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceCapability, EnumPropertyI, mvIMPACT::acquire::TDeviceCapability ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceClass
typedef EnumPropertyI<TDeviceClass> PropertyIDeviceClass;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceClass, EnumPropertyI, mvIMPACT::acquire::TDeviceClass ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceInterfaceLayout
typedef EnumPropertyI<TDeviceInterfaceLayout> PropertyIDeviceInterfaceLayout;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceInterfaceLayout, EnumPropertyI, mvIMPACT::acquire::TDeviceInterfaceLayout ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceLoadSettings
typedef EnumPropertyI<TDeviceLoadSettings> PropertyIDeviceLoadSettings;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceLoadSettings, EnumPropertyI, mvIMPACT::acquire::TDeviceLoadSettings ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceState
typedef EnumPropertyI<TDeviceState> PropertyIDeviceState;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceState, EnumPropertyI, mvIMPACT::acquire::TDeviceState ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TFlatFieldFilterMode
typedef EnumPropertyI<TFlatFieldFilterMode> PropertyIFlatFieldFilterMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIFlatFieldFilterMode, EnumPropertyI, mvIMPACT::acquire::TFlatFieldFilterMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::THWUpdateResult
typedef EnumPropertyI<THWUpdateResult> PropertyIHWUpdateResult;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIHWUpdateResult, EnumPropertyI, mvIMPACT::acquire::THWUpdateResult ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TImageBufferPixelFormat
typedef EnumPropertyI<TImageBufferPixelFormat> PropertyIImageBufferPixelFormat;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIImageBufferPixelFormat, EnumPropertyI, mvIMPACT::acquire::TImageBufferPixelFormat ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TImageDestinationPixelFormat
typedef EnumPropertyI<TImageDestinationPixelFormat> PropertyIImageDestinationPixelFormat;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIImageDestinationPixelFormat, EnumPropertyI, mvIMPACT::acquire::TImageDestinationPixelFormat ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TImageProcessingFilter
typedef EnumPropertyI<TImageProcessingFilter> PropertyIImageProcessingFilter;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIImageProcessingFilter, EnumPropertyI, mvIMPACT::acquire::TImageProcessingFilter ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TRequestImageMemoryMode
typedef EnumPropertyI<TRequestImageMemoryMode> PropertyIRequestImageMemoryMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIRequestImageMemoryMode, EnumPropertyI, mvIMPACT::acquire::TRequestImageMemoryMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TImageRequestControlMode
typedef EnumPropertyI<TImageRequestControlMode> PropertyIImageRequestControlMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIImageRequestControlMode, EnumPropertyI, mvIMPACT::acquire::TImageRequestControlMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLUTGammaMode
typedef EnumPropertyI<TLUTGammaMode> PropertyILUTGammaMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyILUTGammaMode, EnumPropertyI, mvIMPACT::acquire::TLUTGammaMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLUTImplementation
typedef EnumPropertyI<TLUTImplementation> PropertyILUTImplementation;
PYTHON_ONLY( ENUM_PROPERTY( PropertyILUTImplementation, EnumPropertyI, mvIMPACT::acquire::TLUTImplementation ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLUTInterpolationMode
typedef EnumPropertyI<TLUTInterpolationMode> PropertyILUTInterpolationMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyILUTInterpolationMode, EnumPropertyI, mvIMPACT::acquire::TLUTInterpolationMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLUTMapping
typedef EnumPropertyI<TLUTMapping> PropertyILUTMapping;
PYTHON_ONLY( ENUM_PROPERTY( PropertyTLUTMapping, EnumPropertyI, mvIMPACT::acquire::TLUTMapping ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLUTMode
typedef EnumPropertyI<TLUTMode> PropertyILUTMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyILUTMode, EnumPropertyI, mvIMPACT::acquire::TLUTMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TMirrorMode
typedef EnumPropertyI<TMirrorMode> PropertyIMirrorMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIMirrorMode, EnumPropertyI, mvIMPACT::acquire::TMirrorMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TMirrorOperationMode
typedef EnumPropertyI<TMirrorOperationMode> PropertyIMirrorOperationMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIMirrorOperationMode, EnumPropertyI, mvIMPACT::acquire::TMirrorOperationMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TRequestResult
typedef EnumPropertyI<TRequestResult> PropertyIRequestResult;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIRequestResult, EnumPropertyI, mvIMPACT::acquire::TRequestResult ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TRequestState
typedef EnumPropertyI<TRequestState> PropertyIRequestState;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIRequestState, EnumPropertyI, mvIMPACT::acquire::TRequestState ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TScalerMode
typedef EnumPropertyI<TScalerMode> PropertyIScalerMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIScalerMode, EnumPropertyI, mvIMPACT::acquire::TScalerMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TScalerInterpolationMode
typedef EnumPropertyI<TScalerInterpolationMode> PropertyIScalerInterpolationMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIScalerInterpolationMode, EnumPropertyI, mvIMPACT::acquire::TScalerInterpolationMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TThreadPriority
typedef EnumPropertyI<TThreadPriority> PropertyIThreadPriority;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIThreadPriority, EnumPropertyI, mvIMPACT::acquire::TThreadPriority ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TUserDataAccessRight
typedef EnumPropertyI<TUserDataAccessRight> PropertyIUserDataAccessRight;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIUserDataAccessRight, EnumPropertyI, mvIMPACT::acquire::TUserDataAccessRight ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TUserDataReconnectBehaviour
typedef EnumPropertyI<TUserDataReconnectBehaviour> PropertyIUserDataReconnectBehaviour;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIUserDataReconnectBehaviour, EnumPropertyI, mvIMPACT::acquire::TUserDataReconnectBehaviour ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TWhiteBalanceCalibrationMode
typedef EnumPropertyI<TWhiteBalanceCalibrationMode> PropertyIWhiteBalanceCalibrationMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIWhiteBalanceCalibrationMode, EnumPropertyI, mvIMPACT::acquire::TWhiteBalanceCalibrationMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TWhiteBalanceParameter
typedef EnumPropertyI<TWhiteBalanceParameter> PropertyIWhiteBalanceParameter;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIWhiteBalanceParameter, EnumPropertyI, mvIMPACT::acquire::TWhiteBalanceParameter ) )

/// \brief A type for integer properties.
/**
 *  Provided for convenience only. This type represents a standard integer property type.
 */
typedef EnumPropertyI<int> PropertyI;
PYTHON_ONLY( ENUM_PROPERTY( PropertyI, EnumPropertyI, int ) )

//-----------------------------------------------------------------------------
/// \brief A template class to represent 64 bit integer properties and enumerated 64 bit integer properties
/**
 *  The template parameter specifies the enum to use as the value type for this
 *  enumerated property. The class itself however is restricted for integer values.
 *  To use a normal int (not enumerated) properties use the \b mvIMPACT::acquire::PropertyI64 type.
 */
template<class ZYX>
class EnumPropertyI64 : public Property
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::EnumPropertyI64 object.
    explicit EnumPropertyI64() {}
    /// \brief Constructs a new \b mvIMPACT::acquire::EnumPropertyI64 object.
    explicit EnumPropertyI64(
        /// [in] A valid handle to a 64-bit integer property
        HOBJ hProp ) : Property( hProp )
    {
        const TComponentType compType = type();
        if( ( compType != ctPropInt ) && ( compType != ctPropInt64 ) )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_PROP_VALUE_TYPE, hProp );
        }
    }
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    typedef ZYX value_type;
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
#ifndef WRAP_DOTNET
    /// \brief This function queries the property's translation table
    /**
     *  If this property defines a translation table the strings and their corresponding
     *  translation values will be written into \a sequence. If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI64::getTranslationDictString and
     *  \b mvIMPACT::acquire::EnumPropertyI64::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI64& getTranslationDict(
        /// [out] A reference to a container which will receive the data from the properties translation dictionary.
        std::vector<std::pair<std::string, ZYX> >& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        int64_type* pVal = new int64_type[size];
        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT, i;
        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetI64DictEntries( m_hObj, ppBuf, bufSize, pVal, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::pair<std::string, ZYX>( ppBuf[i], static_cast<ZYX>( pVal[i] ) );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;
        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief This function reads a single translation table string entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the string representation of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     */
    std::string getTranslationDictString(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        char* pTranslationString = new char[bufSize];
        while( ( result = OBJ_GetI64DictEntry( m_hObj, pTranslationString, bufSize, 0, index ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            delete [] pTranslationString;
            pTranslationString = new char[bufSize];
        }

        std::string translationString( pTranslationString );
        delete [] pTranslationString;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return translationString;
    }
    /// \brief This function reads a single translation table value entry from a property.
    /**
     *  If this property defines a translation table and \a index specifies a valid entry
     *  the value of this entry will be returned.
     *  If the property does \b NOT define a translation table or \a index specifies
     *  an invalid entry an exception will be thrown.
     *  \sa \b mvIMPACT::acquire::EnumPropertyI64::getTranslationDictValues to query all valid
     *  values efficiently in a single call.
     */
    ZYX getTranslationDictValue(
        /// [in] The index of the entry to read from the property.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        int64_type value;
        if( ( result = OBJ_GetI64DictEntry( m_hObj, 0, 0, &value, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( value );
    }
#ifndef WRAP_DOTNET
    /// \brief This function queries a list of valid strings for this property
    /**
     *  If this property defines a translation table all valid strings will be written into \a sequence.
     *  If the property does \b NOT define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI64::getTranslationDictString multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI64& getTranslationDictStrings(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<std::string>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        unsigned int size = dictSize();

        char** ppBuf = new char* [size];
        size_t bufSize = DEFAULT_STRING_SIZE_LIMIT;
        size_t i = 0;

        for( i = 0; i < size; i++ )
        {
            ppBuf[i] = new char[bufSize];
        }

        while( ( result = OBJ_GetI64DictEntries( m_hObj, ppBuf, bufSize, 0, size ) ) == PROPHANDLING_INPUT_BUFFER_TOO_SMALL )
        {
            bufSize *= BUFFER_INCREMENT_FACTOR;
            for( size_t i = 0; i < size; i++ )
            {
                delete [] ppBuf[i];
                ppBuf[i] = new char[bufSize];
            }
        }

        if( result == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = std::string( ppBuf[i] );
            }
        }

        for( i = 0; i < size; i++ )
        {
            delete [] ppBuf[i];
        }
        delete [] ppBuf;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }

    /// \brief This function queries a list of valid values for this property
    /**
     *  If this property defines a translation table all valid values will be written into \a sequence.
     *  If the property does \b NOT
     *  define a translation table \a sequence will be empty after this function call.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b mvIMPACT::acquire::EnumPropertyI64::getTranslationDictValue multiple times and
     *  therefore this function should be called whenever all entries are required.
     *  \return A const reference to the calling property.
     */
    const EnumPropertyI64& getTranslationDictValues(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence ) const
    {
        TPROPHANDLING_ERROR result = PROPHANDLING_NO_ERROR;
        const unsigned int size = dictSize();
        if( size == 0 )
        {
            sequence.clear();
            return *this;
        }

        int64_type* pVal = new int64_type[size];

        if( ( result = OBJ_GetI64DictEntries( m_hObj, 0, 0, pVal, size ) ) == PROPHANDLING_NO_ERROR )
        {
            sequence.resize( size );
            for( unsigned int i = 0; i < size; i++ )
            {
                sequence[i] = static_cast<ZYX>( pVal[i] );
            }
        }

        delete [] pVal;

        if( result != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return *this;
    }
#endif // WRAP_DOTNET
    /// \brief Reads a value from a property.
    /**
     *  This function queries a single value stored under index \a index in the property.
     *  \return The value stored at \a index within the property.
     */
    ZYX read(
        /// [in] The index of the value to read from the property.
        int index = 0 ) const
    {
        int64_type val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetI64( m_hObj, &val, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return static_cast<ZYX>( val );
    }
    /// \brief Reads the maximum value from a property.
    /**
     *  \note
     *  To find out if the property defines a maximum value \b mvIMPACT::acquire::Property::hasMaxValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a maximum value calling this function will raise
     *  an exception.
     */
    ZYX getMaxValue( void ) const
    {
        return read( plMaxValue );
    }
    /// \brief Reads the minimum value from a property.
    /**
     *  \note
     *  To find out if the property defines a minimum value \b mvIMPACT::acquire::Property::hasMinValue
     *  should be called first.
     *
     *  \note
     *  If the property does not define a minimum value calling this function will raise
     *  an exception.
     */
    ZYX getMinValue( void ) const
    {
        return read( plMinValue );
    }
    /// \brief Reads the step width from a property.
    /**
     *  \note
     *  To find out if the property defines a step width \b mvIMPACT::acquire::Property::hasStepWidth
     *  should be called first.
     *
     *  \note
     *  If the property does not define a step width calling this function will raise
     *  an exception.
     */
    ZYX getStepWidth( void ) const
    {
        return read( plStepWidth );
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        unsigned int valCnt = valuesToRead( start, end );
        sequence.resize( valCnt );
        for( unsigned int i = 0; i < valCnt; i++ )
        {
            sequence[i] = read( i + start );
        }
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to get all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be read one after the other
        bool boAtomic,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        if( boAtomic )
        {
            unsigned int valCnt = valuesToRead( start, end );
            sequence.resize( valCnt );
            int64_type* pValues = new int64_type[valCnt];
            TPROPHANDLING_ERROR result = OBJ_GetI64Array( m_hObj, pValues, valCnt, start );
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                sequence[i] = static_cast<ZYX>( pValues[i] );
            }
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
        }
        else
        {
            read( sequence, start, end );
        }
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     */
    void
#else
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI64&
#endif
    write(
        /// [in] The value to write to the property.
        ZYX value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetI64( m_hObj, static_cast<int64_type>( value ), index ) ) != PROPHANDLING_NO_ERROR )
        {
            std::ostringstream oss;
#if defined(_MSC_VER) && ( _MSC_VER < 1300 ) // is 'old' Microsoft VC 6 compiler?
            oss << static_cast<int>( value ); // this compiler has trouble with 64 bit integer data types as it is quite old... truncate the parameter as it's just part of a exception message anyway
#else
            oss << value;
#endif // #if defined(_MSC_VER) && ( _MSC_VER < 1300 ) // is 'old' Microsoft VC 6 compiler?
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj, oss.str() );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI64&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        int vSize = static_cast<int>( sequence.size() );
        for( int i = 0; i < vSize; i++ )
        {
            write( sequence[i], index + i );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const EnumPropertyI64&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<ZYX>& sequence,
        /// [in] Set this parameter to \a true to set all values at once(recommended). If set to false the function will
        /// need more time and all values in \a sequence will be set one after the other
        bool boAtomic,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        if( boAtomic )
        {
            const unsigned int valCnt = static_cast<unsigned int>( sequence.size() );
            int64_type* pValues = new int64_type[valCnt];
            for( unsigned int i = 0; i < valCnt; i++ )
            {
                pValues[i] = static_cast<int64_type>( sequence[i] );
            }
            TPROPHANDLING_ERROR result = OBJ_SetI64Array( m_hObj, pValues, valCnt, index );
            delete [] pValues;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
#ifndef DOTNET_ONLY_CODE
            return *this;
#endif
        }
        else
        {
            return write( sequence, index );
        }
    }
};

/// Provided for convenience only. This type represents a standard 64 bit integer property type.
typedef EnumPropertyI64<int64_type> PropertyI64;
PYTHON_ONLY( ENUM_PROPERTY( PropertyI64, EnumPropertyI64, mvIMPACT::acquire::int64_type ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceTriggerOverlap
typedef EnumPropertyI64<TDeviceTriggerOverlap> PropertyI64DeviceTriggerOverlap;
PYTHON_ONLY( ENUM_PROPERTY( PropertyI64DeviceTriggerOverlap, EnumPropertyI64, mvIMPACT::acquire::TDeviceTriggerOverlap ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#ifndef WRAP_DOTNET // pointer properties currently not supported under .NET
//-----------------------------------------------------------------------------
/// \brief A class to represent pointer properties.
class PropertyPtr : public Property
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::PropertyPtr object.
    explicit PropertyPtr() {}
    /// \brief Constructs a new \b mvIMPACT::acquire::PropertyPtr object.
    explicit PropertyPtr(
        /// [in] A valid handle to a pointer property
        HOBJ hProp ) : Property( hProp )
    {
        if( type() != ctPropPtr )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_PROP_VALUE_TYPE, hProp );
        }
    }
#   if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    typedef void* value_type;
#   endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
    /// \brief Reads a value from a property.
    /**
     *  This function queries a single value stored under index \a index in the property.
     *  \return The value stored at \a index within the property.
     */
    void* read(
        /// [in] The index of the value to read from the property.
        int index = 0 ) const
    {
        void* val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetP( m_hObj, &val, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return val;
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<void*>& sequence,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        unsigned int valCnt = valuesToRead( start, end );
        sequence.resize( valCnt );
        for( unsigned int i = 0; i < valCnt; i++ )
        {
            sequence[i] = read( i + start );
        }
    }
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     *  \return A const 'self' reference.
     */
    const PropertyPtr& write(
        /// [in] The value to write to the property.
        void* value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetP( m_hObj, value, index ) ) != PROPHANDLING_NO_ERROR )
        {
            std::ostringstream oss;
            oss.setf( std::ios::hex, std::ios::basefield );
            oss << value;
            oss.unsetf( std::ios::hex );
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj, oss.str() );

        }
        return *this;
    }
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *  \return A const 'self' reference.
     */
    const PropertyPtr& write(
        /// [in] An array containing the values to write to the property.
        const std::vector<void*>& sequence,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        unsigned int vSize = static_cast<unsigned int>( sequence.size() );
        for( unsigned int i = 0; i < vSize; i++ )
        {
            write( sequence[i], static_cast<int>( index + i ) );
        }
        return *this;
    }
};
#endif // #ifndef WRAP_DOTNET

//-----------------------------------------------------------------------------
/// \brief A class to represent string properties.
class PropertyS : public Property
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound \b mvIMPACT::acquire::PropertyS object.
    explicit PropertyS() {}
    /// \brief Constructs a new \b mvIMPACT::acquire::PropertyS object.
    explicit PropertyS(
        /// [in] A valid handle to the string property
        HOBJ hProp ) : Property( hProp )
    {
        if( type() != ctPropString )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_INVALID_PROP_VALUE_TYPE, hProp );
        }
    }
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    typedef std::string value_type;
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
    /// \brief Returns the size(in bytes) needed for the binary representation of the string buffer.
    /**
     *  When binary data has been stored in a string property it has been stored in base64 format
     *  internally. If the user want's to know the length of the binary data in decoded format,
     *  this function can be called.
     *
     *  \sa \b mvIMPACT::acquire::PropertyS::readBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::writeBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferMaxSize
     *
     *  \return The size(in bytes) needed for the binary representation of the string buffer.
     */
    unsigned int binaryDataBufferSize(
        /// [in] The index of the value for which to query the
        /// buffer size(if this property holds more than one value).
        int index = 0 ) const
    {
        unsigned int val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetBinaryBufferSize( m_hObj, &val, index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return val;
    }
    /// \brief Reads the max size(in bytes) of binary data this property can store.
    /**
     *  When binary data shall be stored in a string property, the user will need to allocate memory
     *  when this data shall be written to the property. To find out how much binary data can be stored
     *  by a property before calling the function \b mvIMPACT::acquire::PropertyS::writeBinary this function can be used.
     *
     *  \sa \b mvIMPACT::acquire::PropertyS::readBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::writeBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferMaxSize
     *
     *  \return The size(in bytes) needed for the binary representation of the string buffer.
     */
    unsigned int binaryDataBufferMaxSize( void ) const
    {
        unsigned int val;
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_GetBinaryBufferMaxSize( m_hObj, &val ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
        return val;
    }
    /// \brief Reads a value from a property.
    /**
     *  This function queries a single value stored under index \a index in the property.
     *  \return The value stored at \a index within the property.
     */
    std::string read(
        /// [in] The index of the value to read from the property.
        int index = 0 ) const
    {
        return compGetStringParam( sqPropVal, 0, index );
    }
    /// \brief Reads a set of values from a property.
    /**
     *  This function queries a set of values from a property and stores these values
     *  into \a sequence.
     *  \return Nothing.
     */
    void read(
        /// [out] A reference to a container to store the data read from the property into.
        std::vector<std::string>& sequence,
        /// [in] The index from where to start reading values from the property.
        int start = 0,
        /// [in] The index where to stop reading values from the property.
        int end = END_OF_LIST ) const
    {
        unsigned int valCnt = valuesToRead( start, end );
        sequence.resize( valCnt );
        for( unsigned int i = 0; i < valCnt; i++ )
        {
            sequence[i] = read( i + start );
        }
    }
    /// \brief Reads a value stored in the property as binary data.
    /**
     *  Binary data can only be stored in string properties. When writing binary data to a string
     *  property it's stored in base64 format internally. The base64 algorithm converts arbitrary data
     *  into a read and printable string representation. As a result of this 3 bytes of arbitrary binary
     *  data will occupy 4 bytes of memory.
     *
     *  Reading binary data with this function obviously only makes sense if the property has been assigned
     *  the value by a previous call to \b mvIMPACT::acquire::PropertyS::writeBinary (here you find a small code example as well).
     *  However it allows to store any kind of data in a string property.
     *  This can e.g. be interesting when certain data shall be stored in the
     *  user accessible part of the devices non-volatile memory.
     *
     *  \sa \b mvIMPACT::acquire::PropertyS::writeBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferSize, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferMaxSize
     *  \return The binary data representation of the specified property value as a new std::string object.
     */
    std::string readBinary(
        /// [in] The index of the value to get(if this property holds more than one value).
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        unsigned int bufSize = binaryDataBufferSize( index );
        if( bufSize > 0 )
        {
            char* pBuf = new char[bufSize];
            memset( pBuf, 0, bufSize );
            result = OBJ_GetBinary( m_hObj, pBuf, bufSize, index );
            std::string data( pBuf, bufSize );
            delete [] pBuf;
            if( result != PROPHANDLING_NO_ERROR )
            {
                ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
            }
            return data;
        }
        else
        {
            return std::string( "" );
        }
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     */
    void
#else
    /// \brief Writes one value to the property.
    /**
     *  This function writes a single value under index \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const PropertyS&
#endif
    write(
        /// [in] The value to write to the property.
        const std::string& value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        writeS( value, index );
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     */
    void
#else
    /// \brief Writes a set of values to the property.
    /**
     *  This function writes a set of values starting at \a index to the property.
     *
     *  \return A const 'self' reference.
     */
    const PropertyS&
#endif
    write(
        /// [in] An array containing the values to write to the property.
        const std::vector<std::string>& sequence,
        /// [in] The index where to write the first value to the property.
        int index = 0 ) const
    {
        writeS( sequence, index );
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Writes a block of binary data to one entry of the property.
    /**
     *  Binary data can only be stored in string properties. When writing binary data to a string
     *  property it's stored in base64 format internally. The base64 algorithm converts arbitrary data
     *  into a read and printable string representation. As a result of this 3 bytes of arbitrary binary
     *  data will occupy 4 bytes of memory.
     *
     *  Writing binary data with this function allows to store any kind of data in a string property.
     *  This can e.g. be interesting when certain data shall be stored in the
     *  user accessible part of the devices non-volatile memory.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  char binData [6] = { 'A', 'B', 0x00, -128, 'E', 'F' };
     *  const string value(binData, sizeof(binData));
     *  PropertyS prop(getHandleFromSomewhere());
     *  prop.writeBinary( value );
     * \endcode
     *  \endif
     *
     *  To find out if a property contains binary data check if \b mvIMPACT::acquire::cfContainsBinaryData is set e.g. by calling
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  PropertyS prop(getHandleFromSomewhere());
     *  if( prop.flags() & cfContainsBinaryData )
     *  {
     *    // Yes!! Bianry data
     *  }
     * \endcode
     *  \endif
     *
     *  \sa \b mvIMPACT::acquire::PropertyS::readBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferSize, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferMaxSize
     */
    void
#else
    /// \brief Writes a block of binary data to one entry of the property.
    /**
     *  Binary data can only be stored in string properties. When writing binary data to a string
     *  property it's stored in base64 format internally. The base64 algorithm converts arbitrary data
     *  into a read and printable string representation. As a result of this 3 bytes of arbitrary binary
     *  data will occupy 4 bytes of memory.
     *
     *  Writing binary data with this function allows to store any kind of data in a string property.
     *  This can e.g. be interesting when certain data shall be stored in the
     *  user accessible part of the devices non-volatile memory.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  char binData [6] = { 'A', 'B', 0x00, -128, 'E', 'F' };
     *  const string value(binData, sizeof(binData) / sizeof(binData[0] ));
     *  PropertyS prop(getHandleFromSomewhere());
     *  prop.writeBinary( value );
     * \endcode
     *  \endif
     *
     *  To find out if a property contains binary data check if \b mvIMPACT::acquire::cfContainsBinaryData is set e.g. by calling
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  PropertyS prop(getHandleFromSomewhere());
     *  prop.flags() & cfContainsBinaryData.
     * \endcode
     *  \endif
     *
     *  \sa \b mvIMPACT::acquire::PropertyS::readBinary, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferSize, \n
     *      \b mvIMPACT::acquire::PropertyS::binaryDataBufferMaxSize
     *
     *  \return A const 'self' reference.
     */
    const PropertyS&
#endif
    writeBinary(
        /// [in] A const reference to the string holding the binary data that shall
        /// be stored by the this property.
        const std::string& value,
        /// [in] The index defining at which position to write the value.
        int index = 0 ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_SetBinary( m_hObj, value.c_str(), static_cast<unsigned int>( value.size() ), index ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hObj );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
};

//-----------------------------------------------------------------------------
/// \brief A helper class that represents one entry in the devices non-volatile memory
/** (if available).
 *
 *  Each entry of user and device specific data consists of a name, data a property
 *  defining the access rights for this entry and an optional password.
 *
 *  The name can be any string, but currently is limited to 255 characters.
 *
 *  The \b mvIMPACT::acquire::UserDataEntry::data property can hold any type of user data that shall be
 *  stored in the device's non-volatile memory. This can either be a string or binary
 *  data. To store binary data the function
 *  \b mvIMPACT::acquire::PropertyS::writeBinary
 *  can be used. Internally however binary data will be stored as a base64 encoded string.
 *  To read binary data from the property again the function
 *  \b mvIMPACT::acquire::PropertyS::readBinary
 *  must be called. The theoretical limit for the amount of data per entry is 2^16 (64KB) bytes,
 *  however the real limit might be below this value because the device does not offer
 *  that much memory.
 *
 *  The \b mvIMPACT::acquire::UserDataEntry::access property defines the access rights the user has when
 *  working with the entry. \b AFTER creation but \b BEFORE storing the entry into the device
 *  this property can be modified. At this point the creator of the entry can choose between 2 options:
 *  \b mvIMPACT::acquire::udarFull and \b mvIMPACT::acquire::udarRW.
 *  After storing the entry into the device by calling
 *  \b mvIMPACT::acquire::UserData::writeToHardware this property will become \b read-only and
 *  will remain \b read-only until the complete entry is deleted.
 *
 *  When the user creates a new entry for the user accessible device non-volatile memory
 *  he can either create a read/write data set, which means that every user can modify
 *  the \a data and the \a name property. In that case the property \a password is ignored.
 *  Apart from that an entry that can only be modified when the correct password has been
 *  written to the \a password property. To read a value is always allowed.
 *
 *  The \a password property will hold the user input and \b NOT the actual password needed
 *  to unlock the data set. However when an entry has been created the creator afterwards can
 *  write to the password. When the creator then calls the function \b mvIMPACT::acquire::UserData::writeToHardware
 *  the value of the \a password property is used as the 'real' password from that
 *  moment onwards.
 *
 *  The maximum length for the password currently is 255 characters. This does not
 *  provide total security of cause, but the idea of the password only is to hinder
 *  the user from deleting an import entry by accident.
 *
 *  The memory consumed by a single \b mvIMPACT::acquire::UserDataEntry can vary from device
 *  to device and depends on the way the data is stored internally. In any case when an entry
 *  is created it will consume more memory than the amount of bytes written to the
 *  \b mvIMPACT::acquire::UserDataEntry::data. After data has been written to an entry
 *  it is therefore important to check the \b mvIMPACT::acquire::UserData::memoryAvailable_bytes
 *  property if there is still enough memory available.
 *
 *  \note
 *  There might be entries that can't be modified by the user. These entries contain
 *  important data that have been written to the memory during production. An attempt
 *  to delete this data will fail.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  //-----------------------------------------------------------------------------
 *  int storeUserData( Device* pDev, char* pData, size_t dataSize, const std::string& entryName )
 *  //-----------------------------------------------------------------------------
 *  {
 *    UserData& userData = pDev->getUserData();
 *    if( !userData.isAvailable() )
 *    {
 *      cout << "This device does not support user data" << endl;
 *      return -1;
 *    }
 *
 *    cout << "Available user memory(bytes):" << userData.memoryAvailable_bytes.read() << endl;
 *    cout << "Memory consumed already(bytes): " << userData.memoryConsumed_bytes.read() << endl;
 *
 *    // create a new entry
 *    UserDataEntry entry = userData.createEntry();
 *    if( !entry.isValid() )
 *    {
 *      cout << "Failed to create new entry" << endl;
 *      return -2;
 *    }
 *
 *    string value( pData, dataSize );
 *    entry.data.writeBinary( value );
 *    entry.name.write( entryName );
 *
 *    // write to non-volatile memory (therefore the device MUST be closed)
 *    if( pDev->isOpen() )
 *    {
 *      cout << "Can't store data as the device is already in use" << endl;
 *      return -3;
 *    }
 *
 *    userData.writeToHardware();
 *    if( pDev->HWUpdateResult.read() != urSetUserDataWriteOK )
 *    {
 *      cout << "Failed to store data in device non-volatile memory" << endl;
 *      return -4;
 *    }
 *
 *    // now this data is stored in the devices permanent memory
 *    return 0;
 *  }
 * \endcode
 *  \endif
 *
 *  \note
 *  Instances of this class can only be created by the class \b UserData.
 */
class UserDataEntry
//-----------------------------------------------------------------------------
{
    friend class UserData;
    HLIST m_hList;
    //-----------------------------------------------------------------------------
    explicit UserDataEntry( HLIST hList ) : m_hList( hList ), name(), data(),
        access(), password()
        //-----------------------------------------------------------------------------
    {
        if( hList != INVALID_ID )
        {
            ComponentLocator locator( hList );
            locator.bindComponent( name, "Name" );
            locator.bindComponent( data, "Data" );
            locator.bindComponent( access, "Access" );
            locator.bindComponent( password, "Password" );
        }
    }
public:
    /// \brief Call this function to check if this object references an existing entry.
    /**
     *  \return
     *  - true if this object references an existing entry
     *  - false otherwise
     */
    bool isValid( void ) const
    {
        return ( ( m_hList != INVALID_ID ) && ( OBJ_CheckHandle( m_hList, hcmFull ) == PROPHANDLING_NO_ERROR ) );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief The name of the entry.
    /**
     *  The maximum length for the name currently is 255 characters.
     */
    PropertyS name;
    /// \brief The data stored in this entry.
    /**
     *  The theoretical limit for the amount of data per entry is 2^16 (64KB) bytes,
     *  however the real limit might be below this value because the device does not offer
     *  that much memory.
     *
     *  This property can either store string or binary data. See \b mvIMPACT::acquire::PropertyS to find
     *  out how to work with binary data.
     */
    PropertyS data;
    /// \brief The access rights for this entry.
    /**
     *  After this entry has been written to the devices non-volatile memory, this property
     *  will become \b read-only.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TUserDataAccessRight.
     */
    PropertyIUserDataAccessRight access;
    /// \brief The password needed to modify this entry.
    /**
     *  The maximum length for the password is 255 characters currently. When the
     *  \b mvIMPACT::acquire::udarPassword is not specified
     *  by the \a access property, this property will be ignored. Otherwise this
     *  property must have been assigned the correct password (case sensitive) before
     *  the \a name and \a data properties can be modified.
     */
    PropertyS password;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyS getName( void ) const
    {
        return name;
    }
    PropertyS getData( void ) const
    {
        return data;
    }
    PropertyIUserDataAccessRight getAccess( void ) const
    {
        return access;
    }
    PropertyS getPassword( void ) const
    {
        return password;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A helper class to work with the device specific non-volatile memory(if available).
/**
 *  A device might have a certain amount of non-volatile memory which can be accessed and modified by
 *  the user. This data can e.g. be used to store certain parameters or custom data permanently into the
 *  device. Thus this property e.g. allows to store configuration data in the device before shipping it
 *  to the end user.
 *
 *  In theory the number of entries that can be stored in the device is not limited. However no device
 *  will provide an unlimited amount of memory.
 *
 *  Writing data to any property of a \b mvIMPACT::acquire::UserDataEntry as well as creating a new entry by calling
 *  \b mvIMPACT::acquire::UserData::createEntry or deleting an entry with the function
 *  \b mvIMPACT::acquire::UserData::deleteEntry will \b NOT automatically result in
 *  the complete data to be updated in the device specific memory. As writing to the device memory takes
 *  a long (in terms of ms) time depending on the device architecture the modifications will only become.
 *  permanent when calling \b mvIMPACT::acquire::UserData::writeToHardware.
 *
 *  If data at the end of the list of entries doesn't fit in the memory it will be discarded and therefore
 *  will not be visible the next time the device is enumerated/accessed after the running process has
 *  terminated or the device has been disconnected.
 *
 *  \note
 *  Entries that contain no data will not be stored in hardware. If e.g. 5 user data entries have been created,
 *  but only entry 0, 1, 2 and 4 contain data the write operation will pack the data in a way that when the user data
 *  is read from the device again (when the driver for that device is loaded the next time) the list of entries
 *  will contain elements at position 0, 1, 2 and 3 but \b NOT a index number 4. Deleting an entry from the middle
 *  of the list of user data however will \b NOT move all remaining entries one step towards the front of the
 *  list. Therefore during one session the entry indexes will \b NOT change. They can either become invalid (when
 *  the corresponding entry has been deleted), can get valid (newly created entries will always appear at the next
 *  free entry, thus deleting and entry and creating a new directly afterwards will create that entry at exactly the
 *  same index) or new entries can appear (when a new entry has been created and no free list position in the
 *  middle of the list of entries has been available).
 *
 *  \note
 *  The device must be closed in order to write the user data. Check the property \b mvIMPACT::acquire::Device::HWUpdateResult
 *  afterwards in order to make sure the data transfer was successful.
 *
 *  \note
 *  If a device does not offer user accessible data \b mvIMPACT::acquire::UserDataEntry objects will still be returned by certain
 *  functions. To check if this feature is available call \b mvIMPACT::acquire::UserData::isAvailable.
 *
 *  \note
 *  If the user accessible data currently can't be modified an exception of type \b mvIMPACT::acquire::ENoWriteRights
 *  will be thrown.
 *
 *  \note
 *  Instances of this class cannot be constructed directly. To get access the function \b mvIMPACT::acquire::Device::getUserData.
 *  must be used.
 */
class UserData
//-----------------------------------------------------------------------------
{
    friend class Device;
    bool             m_boAvailable;
    HDEV             m_hDev;
    ComponentLocator m_userDataEntryLocator;
    //-----------------------------------------------------------------------------
    explicit UserData( HDEV hDev ) : m_boAvailable( false ), m_hDev( hDev ), m_userDataEntryLocator( hDev ),
        reconnectBehaviour(), memoryAvailable_bytes(), memoryConsumed_bytes()
        //-----------------------------------------------------------------------------
    {
        init();
    }
    /// \brief Allows assignments of \b mvIMPACT::acquire::UserData objects
    UserData& operator=( const UserData& rhs )
    {
        if( this != &rhs )
        {
            m_boAvailable = false;
            m_hDev = rhs.m_hDev;
            m_userDataEntryLocator.bindSearchBase( rhs.m_hDev );
            init();
        }
        return *this;
    }
    void init( void )
    {
        // check if user data is supported
        HOBJ hUserData = m_userDataEntryLocator.findComponent( "UserData" );
        if( hUserData != INVALID_ID )
        {
            Component userData( hUserData );
            // check if the driver supports the new enhanced user data access
            if( userData.isList() )
            {
                m_userDataEntryLocator.bindSearchBase( m_userDataEntryLocator.searchbase_id(), "UserData" );
                m_userDataEntryLocator.bindComponent( reconnectBehaviour, "ReconnectBehaviour" );
                m_userDataEntryLocator.bindComponent( memoryAvailable_bytes, "MemoryAvailable_bytes" );
                m_userDataEntryLocator.bindComponent( memoryConsumed_bytes, "MemoryConsumed_bytes" );
                m_userDataEntryLocator.bindSearchBase( m_userDataEntryLocator.searchbase_id(), "Entries" );
                m_boAvailable = true;
            }
        }
    }
public:
    /// \brief Creates and returns a new entry to store user specific data.
    /**
     *  \note
     *  The data handled by this object will not be stored permanently in the devices non-volatile
     *  memory until \b mvIMPACT::acquire::UserData::writeToHardware has been called successfully.
     *
     *  \sa
     *  \b mvIMPACT::acquire::UserData::deleteEntry, \n
     *     mvIMPACT::acquire::UserData::writeToHardware
     *  \return A new \b mvIMPACT::acquire::UserDataEntry instance that can be used to store user specific data
     *  in the devices non-volatile memory.
     */
    UserDataEntry createEntry( void )
    {
        HLIST hList;
        TDMR_ERROR result = DMR_CreateUserDataEntry( m_hDev, &hList );
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result );
        }
        return UserDataEntry( hList );
    }
    /// \brief Deletes an entry of user specific data.
    /**
     *  If the entry has been stored to the non-volatile memory already and has been assigned
     *  the \b mvIMPACT::acquire::udarPassword flag, this call will fail, when
     *  \b mvIMPACT::acquire::UserDataEntry::password does not contain the correct password.
     *  \note
     *  The data handled by this object will not be removed permanently from the devices non-volatile
     *  memory until \b mvIMPACT::acquire::UserData::writeToHardware( void ) const has been called successfully.
     *  \sa
     *  \b mvIMPACT::acquire::UserData::createEntry, \n
     *      mvIMPACT::acquire::UserData::writeToHardware
     */
    void deleteEntry(
        /// [in] A reference to the entry to delete
        UserDataEntry& userDataEntry )
    {
        TDMR_ERROR result = DMR_DeleteUserDataEntry( m_hDev, userDataEntry.m_hList );
        userDataEntry.m_hList = INVALID_ID;
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result );
        }
    }
    /// \brief Returns the number of bytes of user accessible, non-volatile memory that is still available.
    int getFreeMemory( void ) const
    {
        return memoryAvailable_bytes.read() - memoryConsumed_bytes.read();
    }
    /// \brief Returns An object to work with an existing entry of user specific data.
    /**
     *  \note
     *  If an invalid number has been passed as \a nr an object will be returned as well. Therefore
     *  always check if the entry is valid by calling \b mvIMPACT::acquire::UserDataEntry::isValid before
     *  working with the object.
     *
     *  \return An object to work with an existing entry of user specific data.
     */
    UserDataEntry getUserDataEntry(
        /// [in] The number of the entry to return
        int nr ) const
    {
        std::ostringstream oss;
        oss << "Entry" << nr;
        return UserDataEntry( m_userDataEntryLocator.findComponent( oss.str() ) );
    }
    /// \brief Fills an array with all currently valid user data entry indexes.
    /**
     *  This function can be used to obtain a list of valid parameters for calls to the function \b mvIMPACT::acquire::UserData::getUserDataEntry.
     *  Every value in the returned array will (when passed to the function \b mvIMPACT::acquire::UserData::getUserDataEntry) result in a
     *  valid \b mvIMPACT::acquire::UserDataEntry object.
     */
    void validUserDataEntryIndexes(
        /// [out] A reference to the array receiving the list of valid indexes
        std::vector<int>& sequence ) const
    {
        sequence.clear();
        ComponentIterator it( m_userDataEntryLocator.searchbase_id() );
        if( !it.isValid() )
        {
            return;
        }
        it = it.firstChild();
        if( !it.isValid() )
        {
            return;
        }
        std::string::size_type indexStart = it.name().find_first_of( "0123456789" );
        while( it.isValid() )
        {
            sequence.push_back( atoi( it.name().substr( indexStart ).c_str() ) );
            ++it;
        }
    }
    /// \brief This function should be called to check if this device offers non-volatile memory that can be accessed
    /** by the user.
     *  \return
     *  - true if the device offers non-volatile memory that can be accessed by the user
     *  - false otherwise
     */
    bool isAvailable( void ) const
    {
        return m_boAvailable;
    }
    /// \brief Writes the current set of user data into the devices non-volatile memory.
    /**
     *  This function might take a while. Depending on the device architecture and the amount of
     *  memory up to some hundred ms. Therefore make sure this function is not called more often
     *  than necessary to let your application perform efficiently.
     *  \sa
     *  \b mvIMPACT::acquire::UserData::createEntry, \n
     *  \b mvIMPACT::acquire::UserData::deleteEntry
     */
    void writeToHardware( void ) const
    {
        TDMR_ERROR result = DMR_WriteUserDataToHardware( m_hDev );
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property that can be used to configure how the user data should be treated in case of a device that has been unplugged is plugged back in again.
    /**
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TUserDataReconnectBehaviour.
     *
     *  If a device does not support hot-plugging, this property will be invisible (\b mvIMPACT::acquire::Component::isVisible()),
     */
    PropertyIUserDataReconnectBehaviour reconnectBehaviour;
    /// \brief An integer property \b (read-only) containing the number of bytes of user accessible, non-volatile memory this device has available.
    PropertyI memoryAvailable_bytes;
    /// \brief An integer property \b (read-only) containing the number of bytes of user accessible, non-volatile memory currently consumed by user data.
    /**
     *  This doesn't indicate that all the data has already been stored in the non-volatile memory,
     *  but is the number of bytes needed to store the current user data permanently. No check for
     *  overflows will be performed. If the user defined data exceeds the size of the available memory,
     *  this data will be lost when disconnecting or switching of the supply voltage for this device.
     *
     *  To write data permanently into the device call the function \b mvIMPACT::acquire::UserData::writeToHardware
     *  But even after calling this function data that exceeds the available memory will be lost when the
     *  device looses supply voltage and/or the process terminates.
     */
    PropertyI memoryConsumed_bytes;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIUserDataReconnectBehaviour getReconnectBehaviour( void ) const
    {
        return reconnectBehaviour;
    }
    PropertyI getMemoryAvailable_bytes( void ) const
    {
        return memoryAvailable_bytes;
    }
    PropertyI getMemoryConsumed_bytes( void ) const
    {
        return memoryConsumed_bytes;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief This class and its functions represent an actual device detected by this interface in the current system.
/**
 *
 *  Instances of it can only be created by instances of the \b mvIMPACT::acquire::DeviceManager class
 *  as only the \b mvIMPACT::acquire::DeviceManager has the precise knowledge to do that anyway.
 *  As a result from this fact pointers to instances of \b mvIMPACT::acquire::Device can be obtained
 *  via a \b mvIMPACT::acquire::DeviceManager object only.
 *
 * \attention
 *  \b NEVER try to explicitly delete an instance of \b mvIMPACT::acquire::Device! You did not allocate it
 *  and the result will be a crash! The \b mvIMPACT::acquire::DeviceManager will take care of all resources
 *  for you.
 *
 *  A valid pointer to a \b mvIMPACT::acquire::Device object is needed to construct most of the
 *  other objects available in this interface.
 *
 *  Almost every object requiering a valid pointer to a \b mvIMPACT::acquire::Device object will
 *  need to device in an initialised state as the properties provided e.g. by the
 *  class \b mvIMPACT::acquire::CameraSettingsBase will be constructed when the device is initialised.
 *  To initialise a device this class provides the function \b Device::open.
 *  However every object which needs an initialised device to be constructed
 *  successfully will try to open the device when it hasn't been opened before, so
 *  the user does not need to call this function explicitly.
 *
 *  \attention
 *  Whenever a device is closed via a call to \b mvIMPACT::acquire::Device::close
 *  all other objects constructed with a pointer to that device(e.g. \b mvIMPACT::acquire::CameraSettingsBase
 *  before will become invalid and need to be reconstructed afterwards, so don't close a device needlessly.
 *
 *  \attention
 *  Whenever the last instance to a \b DeviceManager object get's
 *  destroyed within the current process every remaining device that was opened in this process before will
 *  be closed automatically!
 *  Thus every instance to \b mvIMPACT::acquire::Device objects or objects created with a pointer to a
 *  \b mvIMPACT::acquire::Device object will become invalid automatically.
 *  Therefore the user has to make sure there is always at least one instance to a
 *  \b mvIMPACT::acquire::DeviceManager object within the current process unless no more
 *  device access functions shall be called!
 */
class Device
//-----------------------------------------------------------------------------
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    // only the DeviceManager is allowed to create Device objects, as it doesn't make
    // any sense for someone else anyway.
    friend class DeviceManager;
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        HDEV m_hDev;
        HDRV m_hDrv;
        int m_refCnt;
        ReferenceCountedData( HDEV hDev ) : m_hDev( hDev ), m_hDrv( INVALID_ID ), m_refCnt( 1 ) {}
    }* m_pRefData;
    UserData m_userData;
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    void bindPublicProperties( HDEV hDev )
    //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( hDev );
        locator.bindComponent( deviceClass, "DeviceClass" );
        locator.bindComponent( family, "Family" );
        locator.bindComponent( product, "Product" );
        locator.bindComponent( capabilities, "Capabilities" );
        locator.bindComponent( serial, "Serial" );
        locator.bindComponent( state, "State" );
        locator.bindComponent( deviceID, "DeviceID" );
        locator.bindComponent( deviceVersion, "DeviceVersion" );
        locator.bindComponent( firmwareVersion, "FirmwareVersion" );
        locator.bindComponent( loadSettings, "LoadSettings" );
        locator.bindComponent( autoLoadSettingOrder, "AutoLoadSettingOrder" );
        locator.bindComponent( interfaceLayout, "InterfaceLayout" );
        locator.bindComponent( customDataDirectory, "CustomDataDirectory" );
        locator.bindComponent( defaultRequestCount, "DefaultRequestCount" );
        locator.bindComponent( resultQueueCount, "ResultQueueCount" );
        locator.bindComponent( allowUnrecommendedFeatures, "AllowUnrecommendedFeatures" );
        locator.bindComponent( acquisitionStartStopBehaviour, "AcquisitionStartStopBehaviour" );
        locator.bindComponent( HWUpdateResult, "HWUpdateResult" );
        locator.bindComponent( desiredAccess, "DesiredAccess" );
        locator.bindComponent( grantedAccess, "GrantedAccess" );
    }
    //-----------------------------------------------------------------------------
    explicit Device( HDEV hDev ) : m_pRefData( new ReferenceCountedData( hDev ) ), m_userData( hDev ),
        deviceClass(), family(), product(), capabilities(), serial(), state(), deviceID(), deviceVersion(),
        firmwareVersion(), loadSettings(), autoLoadSettingOrder(), interfaceLayout(),
        customDataDirectory(), defaultRequestCount(), resultQueueCount(), allowUnrecommendedFeatures(),
        acquisitionStartStopBehaviour(), HWUpdateResult(), desiredAccess(), grantedAccess()
        //-----------------------------------------------------------------------------
    {
        bindPublicProperties( hDev );
    }
public:
    /// \brief Copy constructor
    /**
     *  Creates a new object from an existing device object. Keep in mind that this new object
     *  will provide access to the very same hardware and therefore you might as well use the original
     *  reference returned from the \b mvIMPACT::acquire::DeviceManager. This constructor
     *  is only provided for internal reference counting to guarantee correct operation of the
     *  objects of this class under all platforms and languages.
     */
    explicit Device( const Device& src ) : m_pRefData( src.m_pRefData ), m_userData( src.m_pRefData->m_hDev ),
        deviceClass( src.deviceClass ), family( src.family ), product( src.product ),
        capabilities( src.capabilities ), serial( src.serial ), state( src.state ),
        deviceID( src.deviceID ), deviceVersion( src.deviceVersion ), firmwareVersion( src.firmwareVersion ), loadSettings( src.loadSettings ),
        autoLoadSettingOrder( src.autoLoadSettingOrder ), interfaceLayout( src.interfaceLayout ),
        customDataDirectory( src.customDataDirectory ), defaultRequestCount( src.defaultRequestCount ), resultQueueCount( src.resultQueueCount ),
        allowUnrecommendedFeatures( src.allowUnrecommendedFeatures ), acquisitionStartStopBehaviour( src.acquisitionStartStopBehaviour ),
        HWUpdateResult( src.HWUpdateResult ), desiredAccess( src.desiredAccess ), grantedAccess( src.grantedAccess )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor
    /**
     *  \note
     *  This destuctor must only be called for objects that have been created directly by
     *  the user on unmanaged heaps. Under most circumstances this means \b NEVER. E.g. for instances
     *  that have been obtained from a \b mvIMPACT::acquire::DeviceManager object do \b NOT call
     *  this destructor. The \b mvIMPACT::acquire::DeviceManager will take care of all the
     *  resources for you.
     */
    ~Device( void )
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::Device objects
    Device& operator=( const Device& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++( m_pRefData->m_refCnt );
            bindPublicProperties( m_pRefData->m_hDev );
            m_userData = UserData( rhs.m_pRefData->m_hDev );
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Closes an opened device.
    /**
     *  This function closes a device previously opened again.
     *
     *  \attention
     *  Whenever a device is closed via a call to \b mvIMPACT::acquire::Device::close
     *  all other objects constructed with a pointer to that device(e.g. \b mvIMPACT::acquire::CameraSettingsBase
     *  before will become invalid and need to be reconstructed afterwards, so don't close a device needlessly.
     *
     *  \sa
     *  \b Device::open, \n \b Device::isOpen
     */
    void close( void )
    {
        if( hDrv() != INVALID_ID )
        {
            DMR_CloseDevice( hDrv(), m_pRefData->m_hDev );
            m_pRefData->m_hDrv = INVALID_ID;
        }
    }
    /// \brief Checks whether this device has a certain capability
    /**
     *  \return
     *  - true if the device has the capability in question
     *  - false otherwise.
     */
    bool hasCapability(
        /// [in] The capability who's presence shall be checked.
        TDeviceCapability capability ) const
    {
        return ( ( capabilities.isValid() ) ? ( ( capabilities.read() & capability ) == capability ) : false );
    }
    /// \brief A unique identifier for this device.
    /**
     *  \note
     *  This handle is \b NOT to be confused with the id stored in the devices EEPROM
     *  The latter one represents a number stored somewhere in the physical device's EEPROM, while
     *  this handle can be assumed as a handle from a software based point of view.
     */
    HDEV hDev( void ) const
    {
        return m_pRefData->m_hDev;
    }
    /// \brief A unique identifier for the functionality offered by this device.
    /**
     *
     *  \note
     *  This identifier is only valid, if the device has been initialised
     *  before by a call to \b Device::open
     */
    HDRV hDrv( void ) const
    {
        if( m_pRefData->m_hDrv == INVALID_ID )
        {
            DMR_GetDriverHandle( m_pRefData->m_hDev, &( m_pRefData->m_hDrv ) );
        }
        return m_pRefData->m_hDrv;
    }
    /// \brief Returns a list providing access to driver library specific features.
    /**
     *  This list does exist only once per device driver library. Changes in this list will affect all
     *  devices that are operated using this device driver library.
     */
    ComponentList deviceDriverFeatureList( void ) const
    {
        HOBJ hObj = INVALID_ID;
        size_t bufSize = sizeof( hObj );
        TDMR_ERROR result = DMR_GetDeviceInfoEx( m_pRefData->m_hDev, dmdithDeviceDriver, &hObj, &bufSize );
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "Failed to query handle of device driver feature list" );
        }
        return ComponentList( hObj );
    }
    /// \brief Returns the current initialisation status in this process.
    /**
     *  If this function returns \a true, this only states that the current
     *  process has not opened the device in question already. A call to \b mvIMPACT::acquire::Device::open()
     *  can still fail because of some other process using this device.
     *
     *  \sa
     *  \b Device::open, \n \b Device::close
     *  \return
     *  - true if the device is initialised(opened) in the current process.
     *  - false otherwise.
     */
    bool isOpen( void ) const
    {
        return hDrv() != INVALID_ID;
    }
    /// \brief Returns the current usage status of this device.
    /**
     *  If this function returns \a true, this device was not in use at the time of calling this function.
     *  A call to \b mvIMPACT::acquire::Device::open() can still fail afterwards if some other process on the system
     *  claimed the device in the time between this call and the call to \b mvIMPACT::acquire::Device::open().
     *
     *  \sa
     *  \b Device::open, \n \b Device::close
     *  \return
     *  - true if the device is in use by at least one process(this includes the calling process).
     *  - false otherwise.
     */
    bool isInUse( void ) const
    {
        if( isOpen() )
        {
            return true;
        }
        unsigned int inUse = 0;
        size_t bufSize = sizeof( inUse );
        TDMR_ERROR result = DMR_GetDeviceInfoEx( m_pRefData->m_hDev, dmditDeviceIsInUse, &inUse, &bufSize );
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "Failed to check for device usage" );
        }
        return inUse != 0;
    }
    /// \brief Opens a device.
    /**
     *  This function will try to open the device represented by this instance of
     *  \b mvIMPACT::acquire::Device. If this fails for any reason an exception will be thrown. The exception
     *  object will contain additional information about the reason for the error.
     *
     *  Calling this function is not really necessary, as each object for accessing
     *  other settings of the device or even the function interface need the device to
     *  be opened in order to be constructed. Therefore all the constructors for these
     *  objects check if the device is open by calling \b mvIMPACT::acquire::Device::isOpen and open
     *  the device if necessary.
     *
     *  \note
     *  Keep in mind that the first object trying to open the device might therefore
     *  throw an exception as well if opening the device fails for some reason.
     *
     *  Whenever a device is opened, the driver executes the following procedure:
     *
     *  <div align="center"><img src="./Device_Setting_Start_Procedure.png" alt="Device_Setting_Start_Procedure.png"></div>
     *  <center>Device setting start procedure</center>
     *
     *  Please have a look <a class="el" href="../GUI_page_mvPropView.html">here</a> for more information about automatic loading of
     *  settings when a device is opened.
     */
    void open( void )
    {
        TDMR_ERROR result = DMR_OpenDevice( m_pRefData->m_hDev, &( m_pRefData->m_hDrv ) );
        if( result != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "Open device failed" );
        }
    }
    /// \brief Assigns a new ID to this device.
    /**
     *  To allow any application to distinguish between different devices of the same
     *  type the user can assign an unique ID to each device using this function. This ID
     *  currently is limited to values between 0 and 250 and is stored in the devices
     *  internal memory. This ID is \b NOT volatile. It will remain stored even if
     *  the device is unplugged.
     *
     *  \note
     *  \b REMARKS:
     *  \note
     *  - The device must be \b closed to assign a new ID.
     *  - This ID is not to be confused with the handle returned by the function \b mvIMPACT::acquire::Device::hDev.
     *  The latter one represents something which can be assumed as a handle from a software based point
     *  of view, while the ID represented by the property \b mvIMPACT::acquire::Device::deviceID is an ID stored in the physical
     *  devices EEPROM.
     *  - Not every device will offer this feature! When calling this function for a device that does not
     *  offer this feature, the function will return \b mvIMPACT::acquire::DMR_FEATURE_NOT_AVAILABLE.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Device::open, \n \b mvIMPACT::acquire::Device::close
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int setID(
        /// [in] The new \a ID that shall be assigned to the device.
        int newID ) const
    {
        return DMR_SetDeviceID( m_pRefData->m_hDev, newID );
    }
    /// \brief Updates the firmware of the device.
    /**
     *  calling this function will cause the driver to download the firmware version compiled
     *  into the driver library into the physical device's EEPROM.
     *
     *  \note
     *  \b REMARKS:
     *  \note
     *  - Be sure what you are doing \b before calling this function. Under normal circumstances
     *  it won't be necessary to update a devices firmware.
     *  - The download will take some time (depending on the device up to 30 seconds). During this time
     *  the device and the thread calling this function will \b NOT respond.
     *  - Do \b NOT interrupt this download.
     *  - After a successful download a USB device needs to be unplugged and plugged in again.
     *  Otherwise the new firmware version will not be activated.
     *  - To download a new firmware version the device must be \b closed.
     *  - Not every device will offer this feature! When calling this function for a device that does not
     *  offer this feature, the function will return \b mvIMPACT::acquire::DMR_FEATURE_NOT_AVAILABLE.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Device::open, \n \b mvIMPACT::acquire::Device::close
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int updateFirmware( void ) const
    {
        return DMR_UpdateFirmware( m_pRefData->m_hDev );
    }
    /// \brief Returns a reference to a helper class to handle user specific data stored in the devices non-volatile memory(if available).
    /**
     *  \return A reference to a helper class to handle user specific data stored in the devices
     *  non-volatile memory(if available).
     */
    UserData& getUserData( void )
    {
        return m_userData;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property \b (read-only) defining the device class this device belongs to.
    /**
     *  \note
     *  This property has been added to the device driver in version 1.8.3 thus might not be available when
     *  working with old device drivers. Therefore always call the function \b mvIMPACT::acquire::Component::isValid
     *  to check if this property is available or not.
     */
    PropertyIDeviceClass deviceClass;
    /// \brief A string property \b (read-only) containing the family name of this device.
    PropertyS family;
    /// \brief A string property \b (read-only) containing the product name of this device.
    PropertyS product;
    /// \brief An enumerated integer property \b (read-only) defining special device capabilities.
    /**
     *  This property allows to query certain device capabillities without opening the device.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceCapability.
     */
    PropertyIDeviceCapability capabilities;
    /// \brief A string property \b (read-only) containing the serial number of this device.
    PropertyS serial;
    /// \brief An enumerated integer property \b (read-only) containing the current state of this device.
    /**
     *  This property e.g. provides information about the current state of the device. For USB devices this can e.g. indicate whether
     *  a device is currently plugged into the system or not.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceState.
     *
     *  \b GenICam/GenTL \b device \b specific:
     *  In order to reduce the amount of network traffic to a minimum, this property will only be updated automatically
     *  for network devices if the property \b mvIMPACT::acquire::Device::registerErrorEvent is set to
     *  \b mvIMPACT::acquire::bTrue (which is the default behaviour). If the IP addresses stay the same the
     *  connection is automatically re-established then once the device is detected by the driver again.
     *  However if the IP address of the device and/or the network adapter of the system it is used from changes
     *  \b mvIMPACT::acquire::DeviceManager::updateDeviceList()
     *  must be called regardless of the value of \b mvIMPACT::acquire::Device::registerErrorEvent before a device
     *  that was lost can re-establish a connection to the capture driver.
     */
    PropertyIDeviceState state;
    /// \brief An integer property \b (read-only) containing the device ID associated with this device.
    /**
     *  A device ID can be used to identify a certain device in the system. The ID is an 8 bit unsigned
     *  integer value stored in the device's EEPROM. In order to allow the
     *  distinct detection of a device via its device ID, the user has to make sure that two devices
     *  belonging to the same product family never share the same ID.
     *
     *  \note
     *  \b REMARKS:
     *  \note
     *  - To modify the device ID a separate tool will be provided. To modify the device
     *  ID without that tool the function \b mvIMPACT::acquire::Device::setID can be called.
     *  - This ID is not to be confused with the handle returned by the function \b mvIMPACT::acquire::Device::hDev.
     *  The latter one represents something which can be assumed as a handle from a software based point
     *  of view, while the ID represented by the property \a deviceID is an ID stored in the physical
     *  devices EEPROM.
     */
    PropertyI deviceID;
    /// \brief A string property \b (read-only) containing the version(e.g. the hard ware revision) of this device.
    /**
     *  \note
     *  This feature is supported by every device driver. If a device does not seem to support this feature (calling \b mvIMPACT::acquire::Component::isValid returns false)
     *  a driver update will fix this.
     */
    PropertyS deviceVersion;
    /// \brief An integer property \b (read-only) containing the firmware version of this device.
    /**
     *  A firmware is considered to be something that is stored in non volatile device memory, thus when a device
     *  is disconnected from the power supply and plugged into a different target system the same code will be executed
     *  on the device.
     *
     *  Some devices however may not support non volatile memory to accommodate a firmware. Typically the device driver then will download
     *  a firmware into the devices RAM at startup. Such devices will be considered as not supporting a firmware as in this
     *  case only the driver version present on the target system has impact on the devices behaviour.
     *
     *  When a device does not support a non volatile firmware, reading this property will always return 0.
     */
    PropertyI firmwareVersion;
    /// \brief An enumerated integer property which can be used to define which previously stored setting to load when the device is opened.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceLoadSettings.
     *
     *  \note
     *  Once the device is in use (has been initialized) this property will become \b (read-only).
     */
    PropertyIDeviceLoadSettings loadSettings;
    /// \brief A string property \b (read-only) containing the auto load order of feature settings that is executed during initialisation of the device.
    /**
     *  The first setting detected on the system will be used.
     */
    PropertyS autoLoadSettingOrder;
    /// \brief An enumerated integer property which can be used to define which interface layout shall be used when the device is opened.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceInterfaceLayout.
     *
     *  \note
     *  Once the device is in use (has been initialized) this property will become \b (read-only).
     *
     *  \note
     *  This feature is available for every device. A device not offering this feature requires a driver update. Always check
     *  for the availability of this feature by calling \b mvIMPACT::acquire::Component::isValid.
     *
     *  Not every device will offer the same options.
     *  Check for valid modes by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIDeviceInterfaceLayout::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIDeviceInterfaceLayout::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIDeviceInterfaceLayout::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyIDeviceInterfaceLayout interfaceLayout;
    /// \brief A string property \b (read-only once the device is open) containing a path to a custom directory for the location of
    /**
     *  camera description files, etc.
     *
     *  When a custom path is assigned to this property this path will be used to locate certain driver specific files
     *  and folders. Under this folder the following structure must be created:
     *
     * \code
     *  <customDataDirectory>
     *      |- CameraFiles  // this folder will be searched by frame grabbers for camera description files
     *      |- GenICam      // this folder will be searched by GenICam compliant devices that refer to a local GenICam&trade; description file
     * \endcode
     *
     *  If this property is left empty
     * \code
     *  %ALL USERS%\Documents\MATRIX VISION\mvIMPACT acquire
     * \endcode
     *  will be used by default on Windows&trade; platforms and
     * \code
     *  /etc/matrix-vision/mvimpact-acquire
     * \endcode
     *  on Linux platforms.
     */
    PropertyS customDataDirectory;
    /// \brief An integer property that defines the number of \b mvIMPACT::acquire::Request objects to be created when the device is opened.
    /**
     *  By writing to this property an application can define the default number of \b mvIMPACT::acquire::Request
     *  objects that will be allocated when opening the device.
     *
     *  \note
     *  This property will become read-only while the device is open.
     */
    PropertyI defaultRequestCount;
    /// \brief An integer property that defines the number of result queues to be created when the device is opened.
    /**
     *  By writing to this property an application can define the number of result queues that will be allocated
     *  when opening the device. The max. value of this property will be the number of video channels offered
     *  by the device. When requesting buffers the user can define in which result queue a processed request shall
     *  end up. When waiting for a request to become ready by calling \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor
     *  the application can specify on which queue to wait.
     *
     *  \note
     *  This property will become read-only while the device is open.
     */
    PropertyI resultQueueCount;
    /// \brief An enumerated integer property which can be used to unlock certain features, that are available but not recommended by the device manufacturer.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  This property will be invisible by default. As the name indicates its use requires special
     *  knowledge that can be provided by the device manufacturer on request.
     *
     *  \note
     *  Once the device is in use (has been initialized) this property will become \b (read-only).
     *
     *  \attention
     *  Unlocking certain unrecommended features might result in the appearance of new translation
     *  dictionary entries for certain properties. Some of these translation strings might contain
     *  an exclamation mark ('!') in its name. \b NEVER rely on that! These exclamation marks
     *  or even the complete entry might disappear with a new driver release without further notice!
     */
    PropertyIBoolean allowUnrecommendedFeatures;
    /// \brief An enumerated integer property defining the start/stop behaviour during acquisition of this driver instance.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAcquisitionStartStopBehaviour.
     *
     *  \note
     *  This property will become read-only while the device is open.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *
     *  Not every device will offer the same options.
     *  Check for valid modes by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIAcquisitionStartStopBehaviour::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIAcquisitionStartStopBehaviour::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIAcquisitionStartStopBehaviour::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyIAcquisitionStartStopBehaviour acquisitionStartStopBehaviour;
    /// \brief An enumerated integer property \b (read-only) defining user executed hardware update results.
    /**
     *  This property e.g. might contain the result of a user executed firmware update.
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::THWUpdateResult.
     */
    PropertyIHWUpdateResult HWUpdateResult;
    /// \brief An enumerated integer property that can be used to define the desired access to the device when opening it.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceAccessMode.
     *
     *  \note
     *  This property will become read-only while the device is open.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIDeviceAccessMode desiredAccess;
    /// \brief An enumerated integer property \b (read-only) that can be used to query the granted access to the device after an attempt to open it.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceAccessMode.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIDeviceAccessMode grantedAccess;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIDeviceClass getDeviceClass( void ) const
    {
        return deviceClass;
    }
    PropertyS getFamily( void ) const
    {
        return family;
    }
    PropertyIDeviceCapability getCapabilities( void ) const
    {
        return capabilities;
    }
    PropertyS getProduct( void ) const
    {
        return product;
    }
    PropertyS getSerial( void ) const
    {
        return serial;
    }
    PropertyIDeviceState getState( void ) const
    {
        return state;
    }
    PropertyI getDeviceID( void ) const
    {
        return deviceID;
    }
    PropertyS getDeviceVersion( void ) const
    {
        return deviceVersion;
    }
    PropertyI getFirmwareVersion( void ) const
    {
        return firmwareVersion;
    }
    PropertyIDeviceLoadSettings getLoadSettings( void ) const
    {
        return loadSettings;
    }
    PropertyS getAutoLoadSettingOrder( void ) const
    {
        return autoLoadSettingOrder;
    }
    PropertyIDeviceInterfaceLayout getInterfaceLayout( void ) const
    {
        return interfaceLayout;
    }
    PropertyI getDefaultRequestCount( void ) const
    {
        return defaultRequestCount;
    }
    PropertyI getResultQueueCount( void ) const
    {
        return resultQueueCount;
    }
    PropertyS getCustomDataDirectory( void ) const
    {
        return customDataDirectory;
    }
    PropertyIBoolean getAllowUnrecommendedFeatures( void ) const
    {
        return allowUnrecommendedFeatures;
    }
    PropertyIAcquisitionStartStopBehaviour getAcquisitionStartStopBehaviour( void ) const
    {
        return acquisitionStartStopBehaviour;
    }
    PropertyIHWUpdateResult getHWUpdateResult( void ) const
    {
        return HWUpdateResult;
    }
    PropertyIDeviceAccessMode getDesiredAccess( void ) const
    {
        return desiredAccess;
    }
    PropertyIDeviceAccessMode getGrantedAccess( void ) const
    {
        return grantedAccess;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Grants access to devices that can be operated by this software interface.
/**
 *  This class will grant access to any device installed on/in the
 *  current system. Whenever somewhere in the code a \b mvIMPACT::acquire::DeviceManager instance is
 *  created it can be used to access any device currently
 *  supported and available. \n \n This is the only class, which is allowed to
 *  create instances of the class \b mvIMPACT::acquire::Device, which are needed to access a certain
 *  device.
 *
 *  If a device is installed in the system but an appropriate driver has not been installed, this class
 *  will \b NOT list these devices.
 *
 *  As a result of this every program written by the use of this interface
 *  will create an instance of \b mvIMPACT::acquire::DeviceManager before performing any other
 *  operations which uses objects or functions from this interface.
 *
 *  During the construction of a \b mvIMPACT::acquire::DeviceManager object the system will be scanned
 *  for supported devices and once the instance has been created the object will
 *  provide an up to date list of devices whenever the user asks for it. Some devices when plugged into
 *  the system after the device manager has been created might require an explicit update of the device
 *  list. This can be triggered by an application by calling \b mvIMPACT::acquire::DeviceManager::updateDeviceList().
 *
 *  This class also provides various functions to find a particular device in the system.
 *  Devices can e.g. be found by family or by serial number.
 *
 *  \note
 *  There always has to be at least one instance of the \b DeviceManager when
 *  the user still works with \b Device objects, as when the last instance to
 *  this object is destroyed all remaining \b Device objects will be closed
 *  automatically!
 *
 * \attention
 *  \b NEVER try to explicitly delete an instance of \b mvIMPACT::acquire::Device! You did not allocate it
 *  and the result will be a crash! The \b mvIMPACT::acquire::DeviceManager will take care of all resources
 *  for you.
 *
 *  \note
 *  A \b mvIMPACT::acquire::DeviceManager object will initially return pointers to \b mvIMPACT::acquire::Device objects,
 *  which all seem to be closed. However one or more of the devices might have been opened
 *  by another instance of the device manager running in a different
 *  process. In that case the attempt to open a device, which seems to be closed will raise
 *  an exception with the error code \b mvIMPACT::acquire::DMR_DRV_ALREADY_IN_USE, which MUST
 *  be handled by the user.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  \b EXAMPLE CODE:
 * \code
 *  //-----------------------------------------------------------------------------
 *  void obtainDevicePointers( void )
 *  //-----------------------------------------------------------------------------
 *  {
 *    // Create an instance of the device manager to get access
 *    // to supported devices.
 *    DeviceManager devMgr;
 *
 *    // show all devices
 *    for( unsigned int i=0; i<devMgr.deviceCount(); i++ )
 *    {
 *      cout << "Dev " << i << ": " << *(devMgr[i]) << endl;
 *    }
 *
 *    // try to find a device using a special criteria
 *    // ( some of these calls will fail!!! )
 *    // will fail as the string is not complete and does
 *    // NOT terminate with a wildcard.
 *    Device* pDev = devMgr.getDeviceBySerial( "SD0*0" );
 *    // might work if there are at least two devices with
 *    // a matching serial number in the system.
 *    pDev = devMgr.getDeviceByFamily( "SD0*0*", 1 );
 *    // might work if there is at least one device whose type
 *    // specifying string starts with 'SampleDevi'.
 *    pDev = devMgr.getDeviceByProduct( "SampleDevi*" );
 *    // will fail as the string is not complete and does NOT
 *    // terminate with the user defined wildcard.
 *    pDev = devMgr.getDeviceByProduct( "SampleD*", 0, '$' );
 *    // will work if there is a device whose product name
 *    // starts with 'SamplePro' and which has
 *    // been assigned the specified device ID.
 *    pDev = devMgr.getDeviceByProductAndID( "SamplePro*", 66, '*' );
 *    // will work if there is at least one device with a serial
 *    // number starting with 'SD00' in the system.
 *    pDev = devMgr.getDeviceBySerial( "SD00*" );
 *    // will return a pointer to the first device detected thus
 *    // this call will work if there is at least one device in the
 *    // current system that is supported by this interface
 *    pDev = devMgr.getDeviceBySerial( "*" );
 *    if( !pDev )
 *    {
 *      cout << "Error! No valid device found" << endl;
 *      return 0;
 *    }
 *
 *    // from here it will be save to work with the pointer returned
 *    // by device manager.
 *    // THE NEXT LINE WILL TRY TO OPEN THE DEVICE! This can fail if the device
 *    // is already running in a different process!
 *    try
 *    {
 *      FunctionInterface func(pDev);
 *      // do some work
 *    }
 *    catch( const ImpactAcquireException& e )
 *    {
 *      // failed to open the device...
 *    }
 *  }
 *
 *  //-----------------------------------------------------------------------------
 *  void showDeviceManagerOperation( void )
 *  //-----------------------------------------------------------------------------
 *  {
 *    Device* pDev = 0;
 *    {
 *      DeviceManager devMgr1;
 *      // will return a pointer to the first device detected thus
 *      // this call will work if there is at least one device in the
 *      // current system that is supported by this interface
 *      pDev = devMgr.getDeviceBySerial( "*" );
 *      if( !pDev )
 *      {
 *        cout << "Error! No valid device found" << endl;
 *        return;
 *      }
 *      // from here it will be save to work with the pointer returned
 *      // by device manager.
 *      // THE NEXT LINE WILL TRY TO OPEN THE DEVICE! This can fail if the device
 *      // is already running in a different process!
 *      try
 *      {
 *        FunctionInterface func(pDev);
 *        // do some work
 *      }
 *      catch( const ImpactAcquireException& e )
 *      {
 *        // failed to open the device...
 *      }
 *      {
 *        DeviceManager devMgr2;
 *        // This will return the same device as pDev
 *        Device* pDev2 = devMgr2.getDeviceBySerial( "*" );
 *      } // here the device manager devMgr2 will be destroyed, but it will NOT close pDev2, as devMgr1 holds a reference to it
 *      Statistics s(pDev); // this will still work
 *    } // here the last device manager will be destroyed, thus from here onwards pDev will be invalid!
 *    Info i(pDev); // this will chrash
 *    DeviceManager devMgr3;
 *    Info i2(devMgr3.getDevice(0)); // this will work again
 *  }
 * \endcode
 *  \endif
 */
class DeviceManager
//-----------------------------------------------------------------------------
{
    typedef std::vector<Device*> DevVector;
    HDMR                    m_deviceBaseList;
    mutable DevVector       m_devVector;
    mutable unsigned int    m_lastChangedCounter;
    int*                    m_pRefCnt;
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( *m_pRefCnt );
        for( unsigned int i = 0; i < m_devVector.size(); i++ )
        {
            delete m_devVector[i];
        }
        // free reference count memory if no one uses it anymore
        if( *m_pRefCnt == 0 )
        {
            delete m_pRefCnt;
        }
    }
    //-----------------------------------------------------------------------------
    Device* getDevice( TDMR_DeviceSearchMode mode, const std::string& search_string, unsigned int devNr, char wildcard ) const
    //-----------------------------------------------------------------------------
    {
        updateInfoVector();
        Device* pResult = 0;
        HDEV device;
        if( DMR_GetDevice( &device, mode, search_string.c_str(), devNr, wildcard ) == DMR_NO_ERROR )
        {
            DevVector::size_type vSize = m_devVector.size();
            for( DevVector::size_type i = 0; i < vSize; i++ )
            {
                if( m_devVector[i]->hDev() == device )
                {
                    pResult = m_devVector[i];
                    break;
                }
            }
        }
        return pResult;
    }
    //-----------------------------------------------------------------------------
    bool updateInfoVector( void ) const
    //-----------------------------------------------------------------------------
    {
        ComponentIterator iter( m_deviceBaseList );
        unsigned int changedCount = iter.changedCounter();
        if( m_lastChangedCounter == changedCount )
        {
            return false;
        }

        m_lastChangedCounter = changedCount;

        unsigned int curDevCnt = 0;
        size_t vSize = m_devVector.size();
        DMR_GetDeviceCount( &curDevCnt );
        if( curDevCnt <= static_cast<unsigned int>( vSize ) )
        {
            return false;
        }

        HDEV hDev = INVALID_ID;
        for( size_t i = vSize; i < curDevCnt; i++ )
        {
            DMR_GetDevice( &hDev, dmdsmSerial, "*", static_cast<unsigned int>( i ), '*' );
            if( hDev != INVALID_ID )
            {
                m_devVector.push_back( new Device( hDev ) );
            }
        }
        return true;
    }
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::DeviceManager object.
    /**
     *  If the construction of this object fails for some reason this constructor
     *  might throw an exception of type \b mvIMPACT::acquire::ImpactAcquireException or a type
     *  derived from this class.
     */
    explicit DeviceManager( void ) : m_deviceBaseList( INVALID_ID ), m_devVector(), m_lastChangedCounter( 0 ), m_pRefCnt( 0 )
    {
        TDMR_ERROR result;
        if( ( result = DMR_Init( &m_deviceBaseList ) ) != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "DMR_Init failed" );
        }
        m_pRefCnt = new int();
        *m_pRefCnt = 1;
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::DeviceManager object from an existing one.
    /**
     *  If the construction of this object fails for some reason this constructor
     *  might throw an exception of type \b mvIMPACT::acquire::ImpactAcquireException or a type
     *  derived from this class.
     */
    explicit DeviceManager( const DeviceManager& src ) : m_deviceBaseList( INVALID_ID ), m_devVector(), m_lastChangedCounter( 0 ), m_pRefCnt( src.m_pRefCnt )
    {
        // we don't need to copy data as the necessary updates are done anyway.
        // But we have to call the underlying C-API to achieve correct reference counting.
        TDMR_ERROR result;
        if( ( result = DMR_Init( &m_deviceBaseList ) ) != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "DMR_Init failed" );
        }
        ++( *m_pRefCnt );
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::DeviceManager objects
    DeviceManager& operator=( const DeviceManager& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefCnt = rhs.m_pRefCnt;
            // inc. the NEW reference count
            ++( *m_pRefCnt );
            // this will fill the device vector with up to date data the next time someone asks
            // for a device.
            m_lastChangedCounter = 0;
            m_devVector.clear();
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Class destructor.
    /**
     *  There always has to be at least one instance of the \b DeviceManager when
     *  the user still works with \b mvIMPACT::acquire::Device objects, as when the last instance to
     *  this object is destroyed all remaining \b mvIMPACT::acquire::Device objects will be closed
     *  automatically!
     */
    virtual ~DeviceManager( void )
    {
        dealloc();
        DMR_Close();
    }
    /// \brief Returns a string containing the version number of the specified library.
    /**
     *  This function returns a string containing the version number of the specified library.
     *
     *  The format of the string will be MAJOR.MINOR.RELEASE.BUILD.
     *  \return A pointer to an internal version string.
     */
    static std::string getVersionAsString(
        /// [in] Specifies the library to query information from.
        TLibraryQuery libraryQuery )
    {
        return DMR_GetVersion( libraryQuery );
    }
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
    /// \brief Returns the internal handle to the device manager created via \b DMR_Init().
    HDMR getInternalHandle( void ) const
    {
        return m_deviceBaseList;
    }
#endif // !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#ifndef WRAP_PYTHON
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::Device object.
    /**
     *  Returns a pointer to a \b mvIMPACT::acquire::Device object specifying the device found at the given
     *  index in the device managers internal list.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of devices detected \n
     *  a STL out_of_range exception is thrown.
     *  \endif
     *
     *  \sa
     *  \b mvIMPACT::acquire::DeviceManager::deviceCount, \n
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceBySerial, \n
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceByFamily, \n
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceByProduct
     *  \return
     *  - A pointer to the device if found.
     *  - an invalid pointer or reference otherwise.
     */
    Device* operator[](
        /// [in] The index of the device to be returned.
        unsigned int index ) const
    {
        return getDevice( index );
    }
#endif // #ifndef WRAP_PYTHON (In Python, use public method getDevice)
    /// \brief Returns the current changed counter for the device list.
    /**
     *  This is a useful function to find out if the device list has been changed in any
     *  way. Such a change might be the appearance of a new USB device or a state change
     *  of any other device (e.g. when a USB device has been unplugged). Thus this function
     *  can be called periodically in order to maintain lists in GUI application for example.
     *  To find out the actual number of devices call \b mvIMPACT::acquire::DeviceManager::deviceCount.
     *  \return The current changed counter for the device list.
     */
    unsigned int changedCount( void ) const
    {
        return Component( m_deviceBaseList ).changedCounter();
    }
    /// \brief Returns the number of devices currently present in the system.
    /**
     *  This function returns the number of devices currently detected in the system. A
     *  device once connected to the system while the device manager was running will remain
     *  in its list even if it's unplugged (then only its state will change). To detect
     *  changes in the \b mvIMPACT::acquire::DeviceManager objects list call the function
     *  \b mvIMPACT::acquire::DeviceManager::changedCount.
     *  \return The number of devices detected in the current system.
     */
    unsigned int deviceCount( void ) const
    {
        updateInfoVector();
        return static_cast<unsigned int>( m_devVector.size() );
    }
    /// \brief Updates the internal device list.
    /**
     *  Most devices can't appear out of nowhere. For example a PCI device is
     *  either connected to the current system when the device manager is initialised
     *  or not but it will never appear at runtime after this instance of \b mvIMPACT::acquire::DeviceManager
     *  has been created.
     *
     *  However certain device classes (e.g. network devices) might be connected to
     *  the system \b AFTER the device manager has been initialised. Some will announce themselves
     *  like e.g. USB devices, which will send a message to every application interessted while others
     *  like e.g. network devices wont. In order not to
     *  pollute the network or bus with constant rescan messages no polling is done inside the driver.
     *  the user should call this function instead when looking for new devices. This can either be done in reasonable intervals or after
     *  it is known that a new device has been connected to the system.
     *
     *  If new devices have been detected a subsequent call to \b mvIMPACT::acquire::DeviceManager::deviceCount
     *  will result in a higher value when compared to a previous call and \b mvIMPACT::acquire::DeviceManager::changedCount
     *  will contain a different value as well then (however this could also happen because a certain device related property did change
     *  its state).
     *
     *  \note
     *  As long as a certain instance of a device manager is active, the devices once detected will \b NOT
     *  disappear from the list of devices even if they have been unplugged from the system. So the list
     *  of devices can only grow, but never gets shorter again until either the process terminates or the
     *  last instance of this class went out of scope. If a device has been unplugged, its \b mvIMPACT::acquire::Device::state
     *  property will change. If the application is interested in getting an instant notification when a
     *  device has been disconnected a callback can be registered on this property. How to do this is explained
     *  here: \ref Callback.cpp
     */
    void updateDeviceList( void ) const
    {
        TDMR_ERROR result = DMR_NO_ERROR;
        if( ( result = DMR_UpdateDeviceList( 0, 0 ) ) != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result );
        }
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::Device object.
    /**
     *  Returns a pointer to a \b mvIMPACT::acquire::Device object specifying the device found at the given
     *  index in the device managers internal list.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of devices detected
     *  a STL out_of_range exception will be thrown.
     *  \endif
     *
     *  \sa
     *  \b DeviceManager::deviceCount, \n
     *  \b DeviceManager::getDeviceBySerial, \n
     *  \b DeviceManager::getDeviceByFamily, \n
     *  \b DeviceManager::getDeviceByProduct
     *  \return
     *  - A pointer to the device if \a index specifies a valid value. \n
     *  - an exception will be raised otherwise.
     */
    Device* getDevice(
        /// [in] The index of the device to be returned.
        unsigned int index ) const
    {
        updateInfoVector();
        return m_devVector.at( index );
    }
    /// \brief Tries to locate a device via the serial number.
    /**
     *  This function tries to find a device by its serial number (or parts of this number).
     *  The user can specify only parts of the serial number and a wildcard. The \b mvIMPACT::acquire::DeviceManager
     *  object will then try to find that matches these parameters in its current list.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  // EXAMPLE
     *  // this will return a pointer to the second device with
     *  // a serial number starting with 'SD' device found or 0
     *  // if there are no 2 sample devices in the system
     *  getDeviceBySerial( "SD*******", 1, '*' );
     *  // will return a pointer to the device with the serial
     *  // number SD000001 if present or 0
     *  getDeviceBySerial( "SD0000001" );
     *  // will return a pointer to the first device in the system
     *  getDeviceBySerial( "*", 0, '*' );
     * \endcode
     *  \endif
     *
     *  \sa
     *  \b DeviceManager::getDeviceByFamily, \n
     *  \b DeviceManager::getDeviceByProduct, \n
     *  \b DeviceManager::getDeviceByProductAndID
     *  \return
     *  - A pointer to the device if found. \n
     *  - an invalid pointer or reference otherwise.
     */
    Device* getDeviceBySerial(
        /// [in] The full serial number or the known parts of the serial number and wildcard characters.
        const std::string& serial = "",
        /// [in] The number of the device to return (if there is more than one candidate).
        unsigned int devNr = 0,
        /// [in] The character to ignore in \a serial.
        char wildcard = '*' ) const
    {
        return getDevice( dmdsmSerial, serial, devNr, wildcard );
    }
    /// \brief Tries to locate a device via the family name.
    /**
     *  This function tries to find a device by its family (or parts of this family name).
     *  The user can specify only parts of the family name and a wildcard. The \b mvIMPACT::acquire::DeviceManager
     *  object will then try to find the device that matches these parameters in its current list.
     *  The family is the most general method of searching for a device apart from 'any device'.
     *  E.g. for a device the family name might be 'SampleDevice'
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  // this will return a pointer to the second 'SampleDevice'
     *  // device found or 0 if there are no 2 'SampleDevice'
     *  // devices in the system
     *  getDeviceByFamily( "SampleD*", 1, '*' );
     *  // will return a pointer to the first device belonging
     *  // to the 'SampleDevice' family if present or 0.
     *  getDeviceByFamily( "SampleDevice" );
     *  // will return the first recognized device in the system
     *  getDeviceByFamily( "*", 0, '*' );
     * \endcode
     *  \endif
     *
     *  \sa
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceBySerial, \n
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceByProduct, \n
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceByProductAndID
     *  \return
     *  - A pointer to the device if found. \n
     *  - an invalid pointer or reference otherwise.
     */
    Device* getDeviceByFamily(
        /// [in] The full family name of the requested device or known parts of it and wildcard characters.
        const std::string& family = "",
        /// [in] The number of the device to return (if there is more than one candidate).
        unsigned int devNr = 0,
        /// [in] The character to ignore in \a family.
        char wildcard = '*' ) const
    {
        return getDevice( dmdsmFamily, family, devNr, wildcard );
    }
    /// \brief Tries to locate a device via the product name.
    /**
     *  This function tries to find a device by its product name (or parts of it).
     *  The user can specify only parts of the name and a wildcard. The \b mvIMPACT::acquire::DeviceManager
     *  object will then try to find the device that matches these parameters in its current list.
     *  The product name is a bit more specific than the family name, but less specific than
     *  the serial. For the 'SampleDevice' for example there might be different product names for different
     *  device types. This might e.g. be 'SampleDevice-G' for a grey version of the sample device and
     *  'SampleDevice-C' for the color version of the sample device.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  // this will return a pointer to the second sample
     *  // device of type 'G' found or 0 if there aren't
     *  // two devices of this type in the system
     *  getDeviceByProduct( "SampleDevice-G", 1 );
     *  // will return a pointer to the first device whose
     *  // product string starts with 'SampleDev' or 0 if
     *  // no such device can be found.
     *  getDeviceByProduct( "SampleDev*", 0, '*' );
     *  // will return the first recognized device in the system
     *  getDeviceByProduct( "*", 0, '*' );
     * \endcode
     *  \endif
     *
     *  \sa
     *  \b DeviceManager::getDeviceBySerial, \n
     *  \b DeviceManager::getDeviceByFamily, \n
     *  \b DeviceManager::getDeviceByProductAndID
     *  \return
     *  - A pointer to the device if found. \n
     *  - an invalid pointer or reference otherwise.
     */
    Device* getDeviceByProduct(
        /// [in] The full product name of the requested device or known parts of it and wildcard characters.
        const std::string& product = "",
        /// [in] The number of the device to return (if there is more than one candidate).
        unsigned int devNr = 0,
        /// [in] The character to ignore in \a product.
        char wildcard = '*' ) const
    {
        return getDevice( dmdsmProduct, product, devNr, wildcard );
    }
    /// \brief Tries to locate a device via the product name and the device ID.
    /**
     *  This function behaves like
     *  \b mvIMPACT::acquire::DeviceManager::getDeviceByProduct
     *  except that the second parameter now is interpreted as the device ID.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  // this will return a pointer to the sample device of
     *  // type 'G' which has been assigned a device ID of '1'
     *  // or 0 if there is no sample device with this ID in the system.
     *  getDeviceByProductAndID( "SampleDevice-G", 1 );
     *  // will return a pointer to the device whose product string
     *  // starts with 'SampleDev' and whose device ID has been set
     *  // to '0' or 0 if no such device can be found.
     *  getDeviceByProductAndID( "SampleDev*", 0, '*' );
     *  // will return the first recognized device with an assigned
     *  // device ID of '0' in the system.
     *  getDeviceByProductAndID( "*", 0 );
     * \endcode
     *  \endif
     *
     *  \sa
     *  \b Device::deviceID, \n
     *  \b DeviceManager::getDeviceByFamily, \n
     *  \b DeviceManager::getDeviceByProduct, \n
     *  \b DeviceManager::getDeviceBySerial, \n
     *  \b Device::setID
     *  \return
     *  - A pointer to the device if found. \n
     *  - an invalid pointer or reference otherwise.
     */
    Device* getDeviceByProductAndID(
        /// [in] The full product name of the requested device or known parts of it and wildcard characters.
        const std::string& product = "",
        /// [in] The ID associated with this device.
        unsigned int devID = 0,
        /// [in] The character to ignore in \a product.
        char wildcard = '*' ) const
    {
        return getDevice( static_cast<TDMR_DeviceSearchMode>( dmdsmProduct | dmdsmUseDevID ), product, devID, wildcard );
    }
};

//-----------------------------------------------------------------------------
/// \brief Defines valid interface list types, which can be located using an instance of \b mvIMPACT::acquire::DeviceComponentLocator.
enum TDeviceListType
//-----------------------------------------------------------------------------
{
    /// \brief A placeholder for an undefined list type
    dltUndefined = dmltUndefined,
    /// \brief Specifies a certain setting.
    /**
     *  An additional string defines the name of the setting to look for.
     */
    dltSetting = dmltSetting,
    /// \brief Specifies the list of driver owned image request objects.
    dltRequest = dmltRequest,
    /// \brief Specifies a certain image request control.
    /**
     *  An additional string defines the name of the setting to look for.
     */
    dltRequestCtrl = dmltRequestCtrl,
    /// \brief Specifies the driver interfaces list containing general information.
    /**
     *  This e.g. can be properties containing the driver version, the current state
     *  of the device and stuff like that.
     */
    dltInfo = dmltInfo,
    /// \brief Specifies the driver interface list containing statistical information.
    /**
     *  This list e.g. might contain the current frame rate, the total number of images
     *  captured, etc.
     */
    dltStatistics = dmltStatistics,
    /// \brief Specifies the driver interface list containing properties, which influence the overall operation of the device.
    /**
     *  This list e.g. might contain the priority of the drivers internal worker thread,
     *  the number of request objects the driver shall work with, etc.
     */
    dltSystemSettings = dmltSystemSettings,
    /// \brief Specifies the driver interface list containing properties to work with any kind of I/O pin belonging to that device.
    /**
     *  Here properties addressing the digital inputs and outputs and other I/O related
     *  properties can be found.
     */
    dltIOSubSystem = dmltIOSubSystem,
    ///  \brief Specifies the driver interface list providing access to the drivers <b>H</b>ardware <b>R</b>eal-<b>T</b>ime <b>C</b>ontroller (HRTC).
    /**
     *  Here properties to control the behaviour of the HRTCs can be found.
     *  \note
     *  This feature might not be available for every device.
     */
    dltRTCtr = dmltRTCtr,
    /// \brief Specifies the driver interface list providing access to the recognized camera description lists.
    /**
     *  Within this list all recognized camera descriptions can be found, each forming a sub list
     *  containing the properties describing the camera.
     *
     *  \note This feature is only available for frame grabber devices currently.
     */
    dltCameraDescriptions = dmltCameraDescriptions,
    /// \brief Specifies the driver interface list providing access to the device specific settings lists.
    /**
     *  \note This feature currently is only available for frame grabber devices.
     */
    dltDeviceSpecificData = dmltDeviceSpecificData,
    /// \brief Specifies the driver interface list providing access to the device specific event type settings lists(<b>deprecated</b>).
    /**
     *  \note Every device will support a different set of events that can be waited for by the user.
     *
     *  This list will contain a sublist for each event type recognized for this device. Within this sublist
     *  all properties that can be describe the current mode an event is operated in user can be found.
     *
     *  \deprecated
     *  This value has been declared deprecated and will be removed in future versions of this interface.
     *  A more flexible way of getting informed about changes in driver features
     *  has been added to the interface and should be used instead. An example for this new method
     *  is \ref Callback.cpp.
     */
    dltEventSubSystemSettings = dmltEventSubSystemSettings,
    /// \brief Specifies the driver interface list providing access to the device specific event type results lists(<b>deprecated</b>).
    /**
     *  \note Every device will support a different set of events that can be waited for by the user.
     *
     *  This list will contain a sublist for each event type recognized for this device. Within this sublist
     *  all result properties that can be queried by the user can be found.
     *
     *  \deprecated
     *  This value has been declared deprecated and will be removed in future versions of this interface.
     *  A more flexible way of getting informed about changes in driver features
     *  has been added to the interface and should be used instead. An example for this new method
     *  is \ref Callback.cpp.
     */
    dltEventSubSystemResults = dmltEventSubSystemResults,
    /// \brief Specifies the driver interface list providing access to the devices memory manager list.
    /**
     *  \note This feature currently is only available for frame grabber devices.
     *
     *  This list will contain properties and lists providing access to settings related to the memory
     *  handling used by the device. E.g. the buffer size for individual DMA blocks can be configured
     *  here.
     *
     *  \note
     *  Properties in this list should only be modified by advanced users.
     */
    dltImageMemoryManager = dmltImageMemoryManager,
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    dltPseudoLast,
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
    /// \brief Defines the last entry in this enumeration.
    /**
     *  This is always equal to the last 'real' enum value.
     */
    dltLast = dltPseudoLast - 1
};

//-----------------------------------------------------------------------------
/// \brief A class to locate components within the driver.
/**
 *  Every driver will offer a set of properties, methods and component lists.
 *  \b mvIMPACT::acquire::Property objects contain data such as the current gain in dB, the state of a
 *  digital input, etc.. \b mvIMPACT::acquire::Method objects can be executed like a normal function and
 *  \b mvIMPACT::acquire::ComponentList objects are used to group certain
 *  objects together to form a logical unit.
 *
 *  When it's necessary to locate one or more of these objects without knowing
 *  exactly where to look for them this locator class can be used to look for the component.
 *
 *  In general the properties interessting for the user can be found in one of the
 *  classes offered by this interface, but when a special property is needed or a
 *  new property that hasn't been embedded into the C++ interface, this class offers
 *  a way to get hold of its handle.
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  // assuming a new property 'MyProp' has been added to the setting of a
 *  // device driver whos product string is 'MyDevice' and it has not been
 *  // added to the corresponding class or the property shall be accessed
 *  // without rebuilding the application.
 *  // The latter situation will require some kind of interpreter that
 *  // can pass the required information to the device driver
 *  //-----------------------------------------------------------------------------
 *  enum TMyEnumType
 *  //-----------------------------------------------------------------------------
 *  {
 *    metOne = 0,
 *    metTwo,
 *    metThree
 *  };
 *
 *  //-----------------------------------------------------------------------------
 *  void fn( void )
 *  //-----------------------------------------------------------------------------
 *  {
 *    DeviceManager devMgr;
 *    Device* pDev = devMgr.getDeviceByProduct( "MyDev*" );
 *    if( pDev )
 *    {
 *      // get access to the base setting
 *      DeviceComponentLocator locator(pDev, dltSetting, "Base");
 *      Property prop;
 *      locator.bindComponent( prop, "MyProp" );
 *      // set the value by string
 *      prop.writeS( "MyVal" );
 *      if( prop.type() == ctPropInt )
 *      {
 *        PropertyI iProp(prop.hObj());
 *        int myVal = 666;
 *        iProp.write( myVal );
 *      }
 *      // if the type is known this test can be omitted:
 *      PropertyF fProp;
 *      locator.bindComponent( fProp, "MyFloatProperty" );
 *      fProp.write( 3.14 );
 *
 *      // When it's an enumerated property this is legal as well:
 *      typedef EnumPropertyI<TMyEnumType> PropertyIMyEnumType;
 *      PropertyIMyEnumType eProp;
 *      locator.bindComponent( eProp, "MyEnumeratedProperty" );
 *      prop.write( metOne );
 *    }
 *  }
 * \endcode
 *  \endif
 *
 */
class DeviceComponentLocator : public ComponentLocatorBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new unbound locator.
    explicit DeviceComponentLocator() : ComponentLocatorBase() {}
    /// \brief Constructs a new bound to the specified base list locator.
    explicit DeviceComponentLocator(
        /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist ) : ComponentLocatorBase( baselist ) {}
    /// \brief Constructs a new locator and searches the search base list.
    explicit DeviceComponentLocator(
        /// [in] A unique identifier to the base list from where to start to search for
        /// the search base.
        HLIST baselist,
        /// [in] The name or path ('/' separated) to the search base.
        const std::string& pathToSearchBase ) : ComponentLocatorBase( baselist, pathToSearchBase ) {}
    /// \brief Contstructs a new locator and bind the search base to the specified list type of the device.
    explicit DeviceComponentLocator(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The type list to locate
        TDeviceListType deviceListType,
        /// [in] A string that can contain the name of the list if there
        /// is more than one candidate (e.g. when looking for settings)
        const std::string& sublistName = "" ) : ComponentLocatorBase()
    {
        bindSearchBaseList( pDev, deviceListType, sublistName );
    }
    /// \brief Assign a new search base to the locator.
    /**
     *  This new search base will be search starting from the base list specified.
     *  \return The unique identifier of the new search base.
     */
    HLIST bindSearchBaseList(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The type list to locate
        TDeviceListType deviceListType,
        /// [in] The name or path to the search base.
        const std::string& sublistName = "" )
    {
        if( !pDev->isOpen() )
        {
            pDev->open();
        }

        HLIST hList;
        TDMR_ERROR result;
        if( ( result = DMR_FindList( pDev->hDrv(), ( sublistName.length() == 0 ) ? 0 : sublistName.c_str(), static_cast<TDMR_ListType>( deviceListType ), 0, &hList ) ) != DMR_NO_ERROR )
        {
            std::ostringstream oss;
            oss << "Couldn't find list '" << sublistName << "'(type: " << deviceListType << ")";
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, oss.str() );
        }
        bindSearchBase( hList );
        return searchbase_id();
    }
};

//-----------------------------------------------------------------------------
/// \brief A wrapper class to handle \b mvIMPACT::acquire::ImageBuffer structures.
/**
 *  This class acts as a simple wrapper to handle \b mvIMPACT::acquire::ImageBuffer structures.
 *  using this class the user is no longer responsible for the memory management
 *  of \b mvIMPACT::acquire::ImageBuffer structures.
 *
 *  Instances of this class are reference counted. Memory is freed once the last instance
 *  to it is destroyed. For details read the corresponding sections in the documentation
 *  of the various constructors and function belonging to this class.
 */
class ImageBufferDesc
//-----------------------------------------------------------------------------
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    friend class Request;
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        int             m_refCnt;
        bool            m_boBufferAllocated;
        ImageBuffer*    m_pBuffer;
        ReferenceCountedData( bool boAllocated = false, ImageBuffer* pBuffer = 0 )
            : m_refCnt( 1 ), m_boBufferAllocated( boAllocated ), m_pBuffer( pBuffer ) {}
        ~ReferenceCountedData()
        {
            if( m_pBuffer )
            {
                if( m_boBufferAllocated )
                {
                    DMR_ReleaseImageBuffer( &m_pBuffer );
                }
                else
                {
                    DMR_ReleaseImageRequestBufferDesc( &m_pBuffer );
                }
            }
        }
        // make sure there are NO assignments or copy constructions by accident!
        ReferenceCountedData& operator=( const ReferenceCountedData& rhs );
        ReferenceCountedData( const ReferenceCountedData& src );
    }* m_pRefData;
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    explicit ImageBufferDesc( bool boAllocated, ImageBuffer* pBuf ) : m_pRefData( new ImageBufferDesc::ReferenceCountedData( boAllocated, pBuf ) ) {}
#endif // DOXYGEN_SHOULD_SKIP_THIS
public:
#ifndef WRAP_DOTNET
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageBufferDesc from an existing one.
    /**
     *  The image buffer associated with this descriptor will not be copied! The internal
     *  data will be reference counted and changing this new object will also affect the
     *  original one. The internal data is freed when the last descriptor referencing the
     *  image is destroyed.
     */
    ImageBufferDesc(
        /// [in] The source buffer descriptor.
        const ImageBufferDesc& source ) : m_pRefData( source.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
#endif // #ifndef WRAP_DOTNET
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageBufferDesc object with \a channelCount channel specific data elements.
    /**
     *  This constructor does \b NOT allocate memory for the actual pixel data but only for the
     *  a structure describing an image.
     */
    explicit ImageBufferDesc(
        /// [in] The number of channel specific data elements to allocate
        /// memory for.
        int channelCount ) : m_pRefData( new ImageBufferDesc::ReferenceCountedData() )
    {
        DMR_AllocImageRequestBufferDesc( &( m_pRefData->m_pBuffer ), channelCount );
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageBufferDesc object and allocates all memory needed.
    /**
     *  This constructor will allocate and handle a new and complete \b ImageBuffer object including
     *  memory for the pixel data for the given pixel format.
     *
     *  The size(in bytes) of allocated pixel memory will depend on the \a pixelFormat as well as on the
     *  parameters \a width and \a height. \b mvIMPACT::acquire::ImageBuffer::iSize will return the
     *  amount of memory allocated for the buffer.
     */
    explicit ImageBufferDesc(
        /// [in] The desired pixel format for the new \b mvIMPACT::acquire::ImageBuffer
        /// object handled by this descriptor.
        TImageBufferPixelFormat pixelFormat,
        /// [in] The width of the new \b mvIMPACT::acquire::ImageBuffer object handled
        /// by this descriptor.
        int width,
        /// [in] The height of the new \b mvIMPACT::acquire::ImageBuffer object handled
        /// by this descriptor.
        int height ) : m_pRefData( new ImageBufferDesc::ReferenceCountedData( true ) )
    {
        DMR_AllocImageBuffer( &( m_pRefData->m_pBuffer ), pixelFormat, width, height );
    }
    /// \brief De-allocates all managed memory again when this is the last object referencing the image.
    ~ImageBufferDesc()
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of two different \b mvIMPACT::acquire::ImageBufferDesc
    /**
     *  The image buffer associated with this descriptor will not be copied! The internal
     *  data will be reference counted and changing this new object will also affect the
     *  original one. The internal data is freed when the last descriptor referencing the
     *  image is destroyed.
     */
    ImageBufferDesc& operator=( const ImageBufferDesc& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            ++( m_pRefData->m_refCnt );
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Returns a deep copy of the \b mvIMPACT::acquire::ImageBuffer object referenced by this descriptor.
    /**
     *  This function will create a deep copy of the \b mvIMPACT::acquire::ImageBuffer object referenced by this
     *  descriptor. Afterwards modifying one of the images will \b NOT influence the other one.
     *
     *  \return A deep copy of the \b mvIMPACT::acquire::ImageBuffer object referenced by this descriptor.
     */
    ImageBufferDesc clone( void ) const
    {
        ImageBuffer* pDst = 0;
        DMR_CopyImageBuffer( m_pRefData->m_pBuffer, &pDst, 0 );
        return ImageBufferDesc( true, pDst );
    }
    /// \brief Grants access to the underlying \b mvIMPACT::acquire::ImageBuffer structure managed by this object.
    /**
     *  \return A pointer to the \b mvIMPACT::acquire::ImageBuffer object referenced by this descriptor.
     */
    ImageBuffer* getBuffer( void ) const
    {
        return m_pRefData->m_pBuffer;
    }
};

//-----------------------------------------------------------------------------
/// \brief Contains information about a captured image.
/**
 *  This class provides access to all sorts of information about the captured image.
 *  Only Instances of \b mvIMPACT::acquire::FunctionInterface are allowed to create objects of \b mvIMPACT::acquire::Request.
 *  Consequently the only way to get access to a \b mvIMPACT::acquire::Request object is via a call to
 *  \b mvIMPACT::acquire::FunctionInterface::getRequest.
 *
 *  A \b mvIMPACT::acquire::Request represents an object used by the driver to fill its internal job
 *  queues. Whenever an image shall be captured a \b mvIMPACT::acquire::Request is sent to the driver
 *  and the hardware then tries to capture the desired image as fast as possible.
 *
 *  Requests are managed by the driver. The only thing the user needs to configure is the
 *  maximum number of requests he wants the driver to work with. To make sure a
 *  certain number of requests are available, the property
 *  \b mvIMPACT::acquire::SystemBase::requestCount can be used.
 *  This can be useful for time critical applications where the hardware can capture
 *  images in the background while the PC performs other tasks. In that case it's
 *  necessary to make sure that the request queue never runs low to ensure loss less
 *  image acquisition.
 */
class Request
//-----------------------------------------------------------------------------
{
    // only the function interface object is allowed to create new requests as
    // it maintains an internal reference to each request created.
    friend class FunctionInterface;
protected:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        int                       m_requestNr;
        mutable ImageBufferDesc   m_imageBufferDesc;
        Device*                   m_pDev;
        DeviceComponentLocator    m_locator;
        ComponentIterator         m_infoIterator;
        std::vector<PropertyI64*> m_chunkCounterValues;
        std::vector<PropertyF*>   m_chunkTimerValues;
        unsigned int              m_refCnt;
        ReferenceCountedData( Device* pDev, int requestNr ) : m_requestNr( requestNr ), m_imageBufferDesc( 1 ), m_pDev( pDev ),
            m_locator(), m_infoIterator(), m_chunkCounterValues(), m_chunkTimerValues(), m_refCnt( 1 )
        {
            if( m_requestNr >= 0 )
            {
                std::ostringstream oss;
                oss << "Entry " << m_requestNr;
                m_locator = DeviceComponentLocator( m_pDev, dltRequest, oss.str() );
            }
            else
            {
                m_locator = DeviceComponentLocator( m_pDev, dltInfo, "CurrentRequestLayout" );
            }
        }
        ~ReferenceCountedData()
        {
            const std::vector<PropertyI64*>::size_type cntcc = m_chunkCounterValues.size();
            for( std::vector<PropertyI64*>::size_type i = 0; i < cntcc; i++ )
            {
                delete m_chunkCounterValues[i];
            }
            const std::vector<PropertyF*>::size_type cntct = m_chunkTimerValues.size();
            for( std::vector<PropertyF*>::size_type j = 0; j < cntct; j++ )
            {
                delete m_chunkTimerValues[j];
            }
        }
        // make sure there are NO assignments or copy constructions by accident!
        ReferenceCountedData( const ReferenceCountedData& src );
        ReferenceCountedData& operator=( const ReferenceCountedData& rhs );
        void collectChunkFeatures( ComponentIterator it )
        {
            while( it.isValid() )
            {
                switch( it.type() )
                {
                case ctList:
                    collectChunkFeatures( it.firstChild() );
                    break;
                case ctPropInt64:
                    {
                        const std::string name( it.name() );
                        if( ( name == "ChunkCounterValue" ) || ( name == "ChunkCounter" ) ) // the latter name is the one that became deprecated with GenICam SFNC 1.5
                        {
                            m_chunkCounterValues.push_back( new PropertyI64( it.hObj() ) );
                        }
                    }
                    break;
                case ctPropFloat:
                    {
                        const std::string name( it.name() );
                        if( ( name == "ChunkTimerValue" ) || ( name == "ChunkTimer" ) ) // the latter name is the one that became deprecated with GenICam SFNC 1.5
                        {
                            m_chunkTimerValues.push_back( new PropertyF( it.hObj() ) );
                        }
                    }
                    break;
                default:
                    break;
                }
                ++it;
            }
        }
        void collectSelectedChunkFeatures( ComponentIterator it )
        {
            if( m_chunkCounterValues.empty() && m_chunkTimerValues.empty() )
            {
                collectChunkFeatures( it );
            }
        }
    }* m_pRefData;
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
private:
    //-----------------------------------------------------------------------------
    void bindPublicProperties( void )
    //-----------------------------------------------------------------------------
    {
        DeviceComponentLocator locator( getComponentLocator() );
        locator.bindComponent( requestResult, "Result" );
        locator.bindComponent( requestState, "State" );
        locator.bindSearchBase( locator.searchbase_id(), "Info" );
        m_pRefData->m_infoIterator = ComponentIterator( locator.searchbase_id() );
        m_pRefData->m_infoIterator = m_pRefData->m_infoIterator.firstChild();
        locator.bindComponent( infoFrameID, "FrameID" );
        locator.bindComponent( infoFrameNr, "FrameNr" );
        locator.bindComponent( infoExposeStart_us, "ExposeStart_us" );
        locator.bindComponent( infoExposeTime_us, "ExposeTime_us" );
        locator.bindComponent( infoTransferDelay_us, "TransferDelay_us" );
        locator.bindComponent( infoGain_dB, "Gain_dB" );
        locator.bindComponent( infoTimeStamp_us, "TimeStamp_us" );
        locator.bindComponent( infoSettingUsed, "SettingUsed" );
        locator.bindComponent( infoImageAverage, "ImageAverage" );
        locator.bindComponent( infoVideoChannel, "VideoChannel" );
        locator.bindComponent( infoCameraOutputUsed, "CameraOutputUsed" );
        locator.bindComponent( infoLineCounter, "LineCounter" );
        locator.bindComponent( infoMissingData_pc, "MissingData_pc" );
        locator.bindComponent( infoIOStatesAtExposureStart, "IOStatesAtExposureStart" );
        locator.bindComponent( infoIOStatesAtExposureEnd, "IOStatesAtExposureEnd" );
        HLIST hList = locator.findComponent( "ChunkData" );
        if( hList != INVALID_ID )
        {
            ComponentLocator chunkFeatureLocator( hList );
            chunkFeatureLocator.bindComponent( chunkOffsetX, "ChunkOffsetX" );
            chunkFeatureLocator.bindComponent( chunkOffsetY, "ChunkOffsetY" );
            chunkFeatureLocator.bindComponent( chunkWidth, "ChunkWidth" );
            chunkFeatureLocator.bindComponent( chunkHeight, "ChunkHeight" );
            chunkFeatureLocator.bindComponent( chunkPixelFormat, "ChunkPixelFormat" );
            chunkFeatureLocator.bindComponent( chunkDynamicRangeMin, "ChunkDynamicRangeMin" );
            chunkFeatureLocator.bindComponent( chunkDynamicRangeMax, "ChunkDynamicRangeMax" );
            chunkFeatureLocator.bindComponent( chunkExposureTime, "ChunkExposureTime" );
            chunkFeatureLocator.bindComponent( chunkTimestamp, "ChunkTimestamp" );
            chunkFeatureLocator.bindComponent( chunkLineStatusAll, "ChunkLineStatusAll" );
            m_pRefData->collectSelectedChunkFeatures( ComponentIterator( hList ).firstChild() );
        }
        locator.bindSearchBase( locator.hObj(), "Image" );
        locator.bindComponent( imageMemoryMode, "MemoryMode" );
        locator.bindComponent( imagePixelFormat, "PixelFormat" );
        locator.bindComponent( imageData, "Data" );
        locator.bindComponent( imageSize, "Size" );
        locator.bindComponent( imageFooter, "Footer" );
        locator.bindComponent( imageFooterSize, "FooterSize" );
        locator.bindComponent( imagePixelPitch, "PixelPitch" );
        locator.bindComponent( imageChannelCount, "ChannelCount" );
        locator.bindComponent( imageChannelOffset, "ChannelOffset" );
        locator.bindComponent( imageChannelBitDepth, "ChannelBitDepth" );
        locator.bindComponent( imageLinePitch, "LinePitch" );
        locator.bindComponent( imageChannelDesc, "ChannelDesc" );
        locator.bindComponent( imageBytesPerPixel, "BytesPerPixel" );
        locator.bindComponent( imageOffsetX, "OffsetX" );
        locator.bindComponent( imageOffsetY, "OffsetY" );
        locator.bindComponent( imageWidth, "Width" );
        locator.bindComponent( imageWidthTotal, "WidthTotal" );
        locator.bindComponent( imageHeight, "Height" );
        locator.bindComponent( imageHeightTotal, "HeightTotal" );
        locator.bindComponent( imageBayerMosaicParity, "BayerMosaicParity" );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
protected:
    explicit Request( Device* pDev, int requestNr ) : m_pRefData( new ReferenceCountedData( pDev, requestNr ) ),
        requestResult(), requestState(),
        infoFrameID(), infoFrameNr(), infoExposeStart_us(),
        infoExposeTime_us(), infoTransferDelay_us(), infoGain_dB(),
        infoTimeStamp_us(), infoSettingUsed(), infoImageAverage(),
        infoVideoChannel(), infoCameraOutputUsed(), infoLineCounter(),
        infoMissingData_pc(), infoIOStatesAtExposureStart(), infoIOStatesAtExposureEnd(),
        chunkOffsetX(), chunkOffsetY(), chunkWidth(), chunkHeight(), chunkPixelFormat(), chunkDynamicRangeMin(),
        chunkDynamicRangeMax(), chunkExposureTime(),  chunkTimestamp(), chunkLineStatusAll(),
        imageMemoryMode(), imagePixelFormat(), imageData(), imageSize(), imageFooter(), imageFooterSize(),
        imagePixelPitch(), imageChannelCount(), imageChannelOffset(), imageChannelBitDepth(), imageLinePitch(),
        imageChannelDesc(), imageBytesPerPixel(), imageOffsetX(), imageOffsetY(), imageWidth(),
        imageWidthTotal(), imageHeight(), imageHeightTotal(), imageBayerMosaicParity()
    {
        bindPublicProperties();
    }
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::Request from and exisiting one.
    explicit Request(
        /// [in] A constant reference to the \b mvIMPACT::acquire::Request object, this object shall be created from
        const Request& src ) : m_pRefData( src.m_pRefData ),
        requestResult( src.requestResult ), requestState( src.requestState ),
        infoFrameID( src.infoFrameID ), infoFrameNr( src.infoFrameNr ), infoExposeStart_us( src.infoExposeStart_us ),
        infoExposeTime_us( src.infoExposeTime_us ), infoTransferDelay_us( src.infoTransferDelay_us ), infoGain_dB( src.infoGain_dB ),
        infoTimeStamp_us( src.infoTimeStamp_us ), infoSettingUsed( src.infoSettingUsed ), infoImageAverage( src.infoImageAverage ),
        infoVideoChannel( src.infoVideoChannel ), infoCameraOutputUsed( src.infoCameraOutputUsed ), infoLineCounter( src.infoLineCounter ),
        infoMissingData_pc( src.infoMissingData_pc ), infoIOStatesAtExposureStart( src.infoIOStatesAtExposureStart ),
        infoIOStatesAtExposureEnd( src.infoIOStatesAtExposureEnd ),
        chunkOffsetX( src.chunkOffsetX ), chunkOffsetY( src.chunkOffsetY ), chunkWidth( src.chunkWidth ), chunkHeight( src.chunkHeight ),
        chunkPixelFormat( src.chunkPixelFormat ), chunkDynamicRangeMin( src.chunkDynamicRangeMin ), chunkDynamicRangeMax( src.chunkDynamicRangeMax ),
        chunkExposureTime( src.chunkExposureTime ), chunkTimestamp( src.chunkTimestamp ), chunkLineStatusAll( src.chunkLineStatusAll ),
        imageMemoryMode( src.imageMemoryMode ), imagePixelFormat( src.imagePixelFormat ),
        imageData( src.imageData ), imageSize( src.imageSize ), imageFooter( src.imageFooter ), imageFooterSize( src.imageFooterSize ),
        imagePixelPitch( src.imagePixelPitch ), imageChannelCount( src.imageChannelCount ), imageChannelOffset( src.imageChannelOffset ),
        imageChannelBitDepth( src.imageChannelBitDepth ), imageLinePitch( src.imageLinePitch ), imageChannelDesc( src.imageChannelDesc ),
        imageBytesPerPixel( src.imageBytesPerPixel ), imageOffsetX( src.imageOffsetX ), imageOffsetY( src.imageOffsetY ), imageWidth( src.imageWidth ),
        imageWidthTotal( src.imageWidthTotal ), imageHeight( src.imageHeight ), imageHeightTotal( src.imageHeightTotal ), imageBayerMosaicParity( src.imageBayerMosaicParity )
    {
        ++( m_pRefData->m_refCnt );
    }
    virtual ~Request()
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::Request objects
    Request& operator=( const Request& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
            bindPublicProperties();
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Returns a compoment locator for this request.
    /**
     * This will allow to write custom feature bind operations.
     * \return A compoment locator for this request.
     */
    DeviceComponentLocator getComponentLocator( void ) const
    {
        return m_pRefData->m_locator;
    }
    /// \brief Returns the number associated with this request.
    int getNumber( void ) const
    {
        return m_pRefData->m_requestNr;
    }
    /// \brief Returns a const reference to the image buffer descriptor of this request.
    /**
     *  This function returns a const reference to the \b mvIMPACT::acquire::ImageBufferDesc associated
     *  with this \b mvIMPACT::acquire::Request.
     *
     *  \warning
     *  Please do \b NEVER work with old references to this structure. So do \b NOT store this reference in
     *  some variable to use it for the evaluation of the next \b mvIMPACT::acquire::Request object as well,
     *  as this will not work. Whenever \b mvIMPACT::acquire::Request::getImageBufferDesc is called the function
     *  will make sure that the data in the returned structure is up to date while when working with
     *  an old reference to the \b mvIMPACT::acquire::ImageBufferDesc structure the containing data might refer
     *  to a previous result.
     */
    const ImageBufferDesc& getImageBufferDesc( void ) const
    {
        DMR_GetImageRequestBuffer( m_pRefData->m_pDev->hDrv(), m_pRefData->m_requestNr, &( m_pRefData->m_imageBufferDesc.m_pRefData->m_pBuffer ) );
        return m_pRefData->m_imageBufferDesc;
    }
#if defined(MVIMPACT_H_) || defined(DOTNET_ONLY_CODE) || defined(DOXYGEN_CPP_DOCUMENTATION)
    /// \brief Returns a mvIMPACT image buffer representation of the image associated with this request.
    /**
     *  \attention
     *  This function is slow, as it performs a copy of the
     *  image data. So using this function will not affect the actual image data from the
     *  device when modifying the image object.
     *
     *  \note
     *  This function is provided for backward compatibility only. New applications should use
     *  the overloaded function accepting a parameter of type \b mvIMPACT::acquire::TImpactBufferFlag.
     *  This function might be faster and provides the user with more control over the way the image
     *  is constructed. However if the \a new function is used also a new driver has to be deployed
     *  on every target system running software compiled with this interface. At least version
     *  1.3.5 of the mvDeviceManager-lib.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  \attention
     *  This function will only be available if \a mvIMPACT.h has been included before including this interface!
     *  \endif
     */
#   ifndef WRAP_DOTNET
    mvIMPACT::
#   endif // #  ifndef WRAP_DOTNET
    Image getIMPACTImage( void ) const
    {
        mvIMPACT_C::IPL_BUFHANDLE hBuf = IPL_DONT_CARE;
        DMR_GetImpactRequestBufferEx( m_pRefData->m_pDev->hDrv(), m_pRefData->m_requestNr, &hBuf, TImpactBufferFlag( 0 ), 0 );
        return
#   ifndef WRAP_DOTNET
            mvIMPACT::
#   endif // #  ifndef WRAP_DOTNET
            Image::imageFactory( hBuf );
    }
    /// \brief Returns a mvIMPACT image buffer representation of the image associated with this request.
    /**
     *  \note
     *  This function might be fast depending on the \a flags parameter or might behave like
     *  \b mvIMPACT::acquire::Request::getIMPACTImage.
     *
     *  Valid values for \a flags are defined by the enumeration \b mvIMPACT::acquire::TImpactBufferFlag.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  \attention
     *  This function will only be available if \a mvIMPACT.h has been included before including this interface!
     *  \endif
     */
#   ifndef WRAP_DOTNET
    mvIMPACT::
#   endif // #  ifndef WRAP_DOTNET
    Image getIMPACTImage(
        /// [in] Flags defining how the buffer is created and behaves.
        TImpactBufferFlag flags ) const
    {
        mvIMPACT_C::IPL_BUFHANDLE hBuf = IPL_DONT_CARE;
        DMR_GetImpactRequestBufferEx( m_pRefData->m_pDev->hDrv(), m_pRefData->m_requestNr, &hBuf, flags, 0 );
        return
#   ifndef WRAP_DOTNET
            mvIMPACT::
#   endif
            Image::imageFactory( hBuf );
    }
#endif // #if defined(MVIMPACT_H_) || defined(DOTNET_ONLY_CODE) || defined(DOXYGEN_CPP_DOCUMENTATION)
    /// \brief Returns an iterator to for iterating inside the info list of the request.
    /**
     *  This can be useful when custom or device specific information has been attached to the request
     *  object that wasn't known at compile time.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  It allows to write code like this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  // 'nr' is assumed to be a valid request number returned from a previous call to
     *  // mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor()
     *  const Request* pRequest = fi.getRequest( nr );
     *  if( pRequest )
     *  {
     *    ComponentIterator it(pRequest->getInfoIterator());
     *    std::map<std::string, std::string> m;
     *    while( it.isValid() )
     *    {
     *      // collect all entries that are properties and store their current name and value in a map
     *      if( it.isProp() && it.isVisible() )
     *      {
     *        Property prop(it);
     *        m.insert( std::make_pair( prop.name(), prop.readS() );
     *      }
     *      ++it;
     *    }
     *    // Here you can work with the information you just obtained
     *  }
     * \endcode
     *  \endif
     * \return
     * An iterator to for iterating inside the info list of the request.
     */
    ComponentIterator getInfoIterator( void ) const
    {
        return m_pRefData->m_infoIterator;
    }
    /// \brief Returns the number of counter values that can be returned as part additional data of the buffer containing the request data.
    /**
     *  A device may support a large number of counters even though just a few of them or none is configured
     *  for counting. In such a case situation when switching on the transmission of the counter values
     *  at the time of the internal frame start event will allow to access the number of counters returned
     *  by this function but then the data in these counters of course will be meaningless.
     *
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     *  \return
     *  The number of counter values that can be returned as part additional data of the buffer containing the request data.
     */
    unsigned int getChunkCounterCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_chunkCounterValues.size() );
    }
#ifndef WRAP_PYTHON // this is currently not available under Python as it would require some work on the SWIG typemaps which will only be done if someone desperately asks for it
    /// \brief Returns a 64 bit integer property \b (read-only) defining the selected couter value at the time of the internal frame start event for this request.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a index is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 getChunkCounterValue(
        /// [in] The index of the counter to return
        unsigned int index ) const
    {
        return PropertyI64( m_pRefData->m_chunkCounterValues.at( index )->hObj() );
    }
#endif // #ifndef WRAP_PYTHON
    /// \brief Returns the number of timer values that can be returned as part additional data of the buffer containing the request data.
    /**
     *  A device may support a large number of timers even though just a few of them or none is configured
     *  to run. In such a case situation when switching on the transmission of the timer values
     *  at the time of the internal frame start event will allow to access the number of timers returned
     *  by this function but then the data in these timers of course will be meaningless.
     *
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    unsigned int getChunkTimerCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_chunkTimerValues.size() );
    }
    /// \brief Returns a 64 bit integer property \b (read-only) defining the selected timer value at the time of the internal frame start event for this request.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a index is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyF getChunkTimerValue(
        /// [in] The index of the timer to return
        unsigned int index ) const
    {
        return PropertyF( m_pRefData->m_chunkTimerValues.at( index )->hObj() );
    }
    /// \brief Convenience function to check if a request has been processed successfully.
    /**
     *  This is just a nicer way of checking the value of the \a requestResult property:
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  It allows to write code like this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  // 'nr' is assumed to be a valid request number returned from a previous call to
     *  // mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor()
     *  const Request* pRequest = fi.getRequest( nr );
     *  if( pRequest->isOK() )
     *  {
     *    // do something
     *  }
     * \endcode
     *
     *  Instead if this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  // 'nr' is assumed to be a valid request number returned from a previous call to
     *  // mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor()
     *  const Request* pRequest = fi.getRequest( nr );
     *  if( pRequest->requestResult.read() == rrOK )
     *  {
     *    // do something
     *  }
     * \endcode
     *  \endif
     *  \return  \n
     *  - true if the request has been processed successfully.
     *  - false otherwise.
     */
    bool isOK( void ) const
    {
        return ( requestResult.read() == rrOK );
    }
    /// \brief Allows to set a request into configuration mode.
    /**
     *  In configuration mode certain properties like \b mvIMPACT::acquire::Request::imageData,
     *  \b mvIMPACT::acquire::Request::imageSize, \b mvIMPACT::acquire::Request::imageMemoryMode
     *  of a request object can be modified. This can be used to configure one or more requests
     *  to use a user supplied memory. To use only a subset of the \b mvIMPACT::acquire::Request
     *  objects available the \b mvIMPACT::acquire::ImageRequestControl::requestToUse feature
     *  can be used.
     *
     *  Only requests that are currently not used by the driver and are not locked
     *  because they contain image data that hasn't been processed can be set into
     *  configuration mode.
     *
     *  \note
     *  Instead of calling this function directly for most cases it is much more convenient to use
     *  the functions \b mvIMPACT::acquire::Request::attachUserBuffer and \b mvIMPACT::acquire::Request::detachUserBuffer
     *  instead.
     *
     *  User supplied buffers must follow the alignment and size requirements reported by versions of the
     *  function \b mvIMPACT::acquire::FunctionInterface::getCurrentCaptureBufferLayout. Calling
     *  a version of this function will return all the information required to allocate buffers that
     *  can be used to capture data for the specified settings.
     *
     *  When allocating memory on the heap, the complete buffer size is needed which is calculated like
     *  this:
     *
     *  \b size + alignment
     *
     *  \note
     *  The address passed to the request object must be aligned already!
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  Device* pDev = getDevicePointerFromSomewhere()
     *  FunctionInterface fi(pDev);
     *  Request* pRequest = fi.getRequest( 0 );
     *  // the buffer assigned to the request object must be aligned accordingly
     *  // the size of the user supplied buffer MUST NOT include the additional size
     *  // caused by the alignment
     *  if( pRequest->attachUserBuffer( getAlignedMemoryPtr(), getAlignedMemorySize() ) ) == DMR_NO_ERROR )
     *  {
     *    ImageRequestControl irc(pDev);
     *    irc.requestToUse.write( 0 ); // use the buffer just configured for the next image request
     *    // now the next image will be captured into the user supplied memory
     *    fi.imageRequestSingle( &irc ); // this will send request '0' to the driver
     *    // wait for the buffer. Once it has been returned by the driver AND the user buffer shall no
     *    // longer be used call
     *    if( pRequest->detachUserBuffer() != DMR_NO_ERROR )
     *    {
     *      // handle error
     *    }
     *    // now this request will use internal memory again.
     *  }
     *  else
     *  {
     *     // handle error
     *  }
     * \endcode
     *  \endif
     *  \note
     *  A request that is in configuration mode can't be sent to the driver for acquisition until
     *  \b mvIMPACT::acquire::Request::unlock or \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock has been called again.
     *  By using \b mvIMPACT::acquire::Request::attachUserBuffer and \b mvIMPACT::acquire::Request::detachUserBuffer this locking
     *  and unlocking is done internally thus the application does not need to worry about this.
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::Request::attachUserBuffer, \n
     *  \b mvIMPACT::acquire::Request::detachUserBuffer, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestReset
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int configure( void )
    {
        return DMR_ImageRequestConfigure( m_pRefData->m_pDev->hDrv(), m_pRefData->m_requestNr, 0, 0 );
    }
    /// \brief Unlocks the request for the driver again.
    /**
     *
     *  To ensure that no image data is overwritten by another image request while the user still
     *  deals with the image from a previous acquisition each image buffer will be locked by the driver when
     *  it is returned to the user by a call to \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor.
     *  No new image will be captured into the same buffer until the user unlocks the buffer again
     *  by calling \b mvIMPACT::acquire::Request::unlock or \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock.
     *
     *  \note
     *  After unlocking a request it is no longer guaranteed that the memory once referenced by the
     *  request and the image buffer belonging to it stays valid, so do \b NEVER try to access
     *  memory belonging to an unlocked request object. If you need to copy the image buffer or
     *  modify it in any other way, do everything you have to do \b BEFORE calling this function!
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestReset, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int unlock( void )
    {
        return DMR_ImageRequestUnlock( m_pRefData->m_pDev->hDrv(), m_pRefData->m_requestNr );
    }
    /// \brief Convenience function to attach a user supplied buffer to a \b mvIMPACT::acquire::Request object
    /**
     *  This function just provides a nicer way to attach a user supplied buffer to a \b mvIMPACT::acquire::Request.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  It e.g. allows to write code like this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  const Request* pRequest = fi.getRequest( 0 ); // get access to request 0
     *  pRequest->attachUserBuffer( getBufferFromSomewhere(), getBufferSizeFromSomewhere() );
     * \endcode
     *
     *  Instead if this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  const Request* pRequest = fi.getRequest( 0 ); // get access to request 0
     *  if( pRequest->configure() == DMR_NO_ERROR )
     *  {
     *    pRequest->imageMemoryMode.write( rimmUser );
     *    pRequest->imageData.write( getBufferFromSomewhere() );
     *    pRequest->imageSize.write( getBufferSizeFromSomewhere() );
     *    pRequest->unlock();
     *  }
     * \endcode
     *  \endif
     *
     *  To find out more about capturing to user supplied buffers please refer to the example
     *  \ref CaptureToUserMemory.cpp and have a look at the
     *  documentation of the function \b mvIMPACT::acquire::Request::configure
     *
     *  \sa
     *  \b mvIMPACT::acquire::Request::detachUserBuffer, \n
     *     mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *     mvIMPACT::acquire::Request::unlock, \n
     *     mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *     mvIMPACT::acquire::Request::configure
     *  \return  \n
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int attachUserBuffer(
        /// [in] A pointer to a user supplied buffer
        void* pBuf,
        /// [in] The size of the user supplied buffer
        int bufSize )
    {
        int result = configure();
        if( result == DMR_NO_ERROR )
        {
            imageMemoryMode.write( rimmUser );
            imageData.write( pBuf );
            imageSize.write( bufSize );
            result = unlock();
        }
        return result;
    }
    /// \brief Convenience function to detach a user supplied buffer from a \b mvIMPACT::acquire::Request object
    /**
     *  This function just provides a nicer way to detach a user supplied buffer from a \b mvIMPACT::acquire::Request.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  It e.g. allows to write code like this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  const Request* pRequest = fi.getRequest( 0 ); // get access to request 0
     *  pRequest->detachUserBuffer();
     * \endcode
     *
     *  Instead if this:
     *
     * \code
     *  // 'fi' is assumed to be a valid instance of mvIMPACT::acquire::FunctionInterface,
     *  const Request* pRequest = fi.getRequest( 0 ); // get access to request 0
     *  if( pRequest->configure() == DMR_NO_ERROR )
     *  {
     *    pRequest->imageMemoryMode.write( rimmAuto );
     *    pRequest->unlock();
     *  }
     * \endcode
     *  \endif
     *
     *  To find out more about capturing to user supplied buffers please refer to the example
     *  \ref CaptureToUserMemory.cpp and have a look at the
     *  documentation of the function \b mvIMPACT::acquire::Request::configure
     *
     *  \sa
     *  \b mvIMPACT::acquire::Request::attachUserBuffer, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure
     *  \return  \n
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int detachUserBuffer( void )
    {
        int result = configure();
        if( result == DMR_NO_ERROR )
        {
            imageMemoryMode.write( rimmAuto );
            result = unlock();
        }
        return result;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property \b (read-only) defining the result of this request.
    /**
     *  This parameter indicates whether a previous image acquisition has been
     *  successful or not.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TRequestResult.
     */
    PropertyIRequestResult requestResult;
    /// \brief An enumerated integer property \b (read-only) defining the current state of this request.
    /**
     *  This parameter indicates the current state of this request. A \b mvIMPACT::acquire::Request e.g.
     *  can currently be idle. This would mean that it is currently not used for
     *  image acquisition. Also a \b mvIMPACT::acquire::Request can be in 'Capturing' state, which means
     *  it is currently processed by the driver.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TRequestState.
     */
    PropertyIRequestState requestState;
    /// \brief A 64 bit integer property \b (read-only) containing a frame identifier.
    /**
     *  This parameter is returned as part of the \b mvIMPACT::acquire::Request. It is
     *  used to associate a certain image with a unique identifier.
     *
     *  When an \b mvIMPACT::acquire::RTCtrProgramStep instruction of the type \b mvIMPACT::acquire::rtctrlProgTriggerSet
     *  is executed, the \a frameID is set to the value of the property \a FrameID of a corresponding \b mvIMPACT::acquire::RTCtrProgramStep if the
     *  HRTC is used and any program step executes the setting of the ID.
     *
     *  If a device (e.g. a digital camera) supports the transmission of a unique identifier for an image, this property will reflect
     *  the identifier as transmitted by the device.
     *
     *  \note \b mvBlueFOX, \b mvBlueCOUGAR-P, \b mvBlueLYNX-M7 and \b mvHYPERION specific:
     *  This property will only contain meaningful data if the device supports at least one \b HRTC and a program is running and writing data to
     *  the property OR if a device supports the transmission of a unique identifier for an image. Currently HRTC programs are supported by \b mvBlueFOX, \b mvBlueCOUGAR-P, \b mvBlueLYNX-M7
     *  and \b mvHYPERION devices while a unique identifier transmitted by the device is support by all GigE Vision compliant devices.
     *  For GigE Vision compliant devices, this will be a 16 bit counter that will be incremented with every data block(e.g. an image or chunk data) transmitted
     *  by the device where '0' is a reserved value, which will never be used, thus values like this will be transmitted in a round robin scheme:
     *  1, 2, 3, ..., 65535, 1, 2, ... Please note that due to the nature of \b mvIMPACT::acquire::amContinuous there might be gaps between the
     *  last image of a continuous stream and the first image of the next continuous stream if the device is stopped or re-programmed in between.
     *
     *  \note \b GigE \b Vision and \b USB3 \b Vision specific:
     *  This is the property where the so called block ID will be stored.
     */
    PropertyI64 infoFrameID;
    /// \brief A 64 bit integer property \b (read-only) containing the number of images captured since the driver has been initialised including the current image.
    PropertyI64 infoFrameNr;
    /// \brief An integer property \b (read-only) containing a timestamp (in us) defining the time the device started the exposure of the image associated with this \b mvIMPACT::acquire::Request object.
    /**
     *  This value will stay 0 if nothing is known about the time the exposure did start.
     *  In such a case the \b mvIMPACT::acquire::Request::infoTimeStamp_us property should be used instead.
     */
    PropertyI infoExposeStart_us;
    /// \brief An integer property \b (read-only) containing the 'real' expose time (in us) used to generate this image.
    /**
     *  This might differ slightly from the value selected by the user via
     *  the corresponding exposure property depending on the precision
     *  available for the device or the connected camera.
     */
    PropertyI infoExposeTime_us;
    /// \brief An integer property \b (read-only) containing the time the transaction of this image has been delayed (in us) because either the bus was blocked or the CPU was busy.
    /**
     *  Normally this value will be 0. A value larger than 0 indicates that the system
     *  can't manage the current load.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyI infoTransferDelay_us;
    /// \brief A float property \b (read-only) containing the gain(in dB) this image has been taken with.
    PropertyF infoGain_dB;
    /// \brief A 64 bit integer property \b (read-only) containing a timestamp to define the exact time this image has been captured
    /// (stored after the integration). The timestamp is independent from the FPGA and has a resolution of 1us.
    ///
    /// \b mvBlueFOX specific:
    /// The counter of the timestamp starts when the camera gets initialized. It is measured in us.
    PropertyI64 infoTimeStamp_us;
    /// \brief A enumerated integer property \b (read-only) containing the setting that was used for processing this request.
    /**
     *  The string representation will be a valid setting name, the integer representation can be casted into a \b mvIMPACT::acquire::HOBJ type.
     */
    PropertyI infoSettingUsed;
    /// \brief Currently unsupported.
    PropertyF infoImageAverage;
    /// \brief An integer property \b (read-only) containing the video input channel of the device this image has been acquired from.
    /**
     *  \note
     *  This property is not supported by every device and will only be available if the device has more than one input channel.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyI infoVideoChannel;
    /// \brief An enumerated integer property \b (read-only) containing the camera output used to transmit the image to the capture device.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraOutput.
     *
     *  \note This property is not supported by every device. Therefore always call the function \b mvIMPACT::acquire::Component::isValid
     *  to check if this property is available or not.
     */
    PropertyICameraOutput infoCameraOutputUsed;
    /// \brief An integer property \b (read-only) containing the number of lines since the last trigger event.
    /**
     *  Will contain
     *  - the number of lines since last trigger event of the first line of the snap if line counting is enabled
     *  - -1 otherwise
     *
     *  \note This property is not supported by every device. Therefore always call the function \b mvIMPACT::acquire::Component::isValid
     *  to check if this property is available or not.
     */
    PropertyI infoLineCounter;
    /// \brief A float property \b (read-only) containing the amount of data missing in the current image.
    /**
     *  The value of this property will be 0 almost always. However if a device
     *  can detect blocks of missing data and an image request has returned with \b mvIMPACT::acquire::rrFrameIncomplete
     *  to indicate that not all the data has been captured, this property will contain the amount of data missing in percent.
     */
    PropertyF infoMissingData_pc;
    /// \brief An integer property \b (read-only) containing the state of all the IO lines at the beginning of the frame exposure as a device specific bit field.
    /**
     *  The following table documents which bit in this property represents the state of which digital I/O for \b mvBlueLYNX-M7 devices:
     *
     *  <table>
     *  <tr><td class="header">bit</td><td class="header">IO</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">out 0</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">out 1</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">in 0</td></tr>
     *  <tr><td class="indexvalue">3</td><td class="indexvalue">in 1</td></tr>
     *  <tr><td class="indexvalue">4</td><td class="indexvalue">out 2</td></tr>
     *  <tr><td class="indexvalue">5</td><td class="indexvalue">out 3</td></tr>
     *  <tr><td class="indexvalue">6</td><td class="indexvalue">in 2</td></tr>
     *  <tr><td class="indexvalue">7</td><td class="indexvalue">in 3</td></tr>
     *  <tr><td class="indexvalue">8</td><td class="indexvalue">in 4</td></tr>
     *  <tr><td class="indexvalue">9</td><td class="indexvalue">out 4</td></tr>
     *  <tr><td class="indexvalue">10</td><td class="indexvalue">out 5</td></tr>
     *  </table>
     *
     *  The following table documents which bit in this property represents the state of which digital I/O for \b mvBlueCOUGAR-P devices:
     *
     *  <table>
     *  <tr><td class="header">bit</td><td class="header">IO</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">out 0</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">out 1</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">in 0</td></tr>
     *  <tr><td class="indexvalue">3</td><td class="indexvalue">in 1</td></tr>
     *  <tr><td class="indexvalue">4</td><td class="indexvalue">out 2</td></tr>
     *  <tr><td class="indexvalue">5</td><td class="indexvalue">out 3</td></tr>
     *  </table>
     *
     *  The following table documents which bit in this property represents the state of which digital I/O for \b mvBlueFOX devices:
     *
     *  <table>
     *  <tr><td class="header">bit</td><td class="header">IO</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">in 0</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">in 1</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">in 2(if available)</td></tr>
     *  <tr><td class="indexvalue">3</td><td class="indexvalue">in 3(if available)</td></tr>
     *  <tr><td class="indexvalue">4</td><td class="indexvalue">out 0</td></tr>
     *  <tr><td class="indexvalue">5</td><td class="indexvalue">out 1</td></tr>
     *  <tr><td class="indexvalue">6</td><td class="indexvalue">out 2(if available)</td></tr>
     *  <tr><td class="indexvalue">7</td><td class="indexvalue">out 3(if available)</td></tr>
     *  </table>
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  #include <sstream>
     *  #include <mvIMPACT/mvIMPACT_acquire.h>
     *
     *  using namespace std;
     *  using namespace mvIMPACT::acquire;
     *
     *  //-----------------------------------------------------------------------------
     *  template<typename _Ty>
     *  string decToBinString( _Ty value )
     *  //-----------------------------------------------------------------------------
     *  {
     *    const size_t bitCount = sizeof(_Ty)*8;
     *    ostringstream oss;
     *    for( size_t i=0; i<bitCount; i++ )
     *    {
     *      oss << ( ( value & ( 1 << ( bitCount - 1 - i ) ) ) ? "1" : "0" );
     *    }
     *    return oss.str();
     *  }
     *
     *  //-----------------------------------------------------------------------------
     *  void printIOStateOfAValidRequest( const Request* pRequest )
     *  //-----------------------------------------------------------------------------
     *  {
     *    if( pRequest->infoIOStatesAtExposureStart.isValid() )
     *    {
     *       cout << pRequest->infoIOStatesAtExposureStart.name() << ": " << decToBinString( pRequest->infoIOStatesAtExposureStart.read() ) << endl;
     *    }
     *  }
     * \endcode
     *  \endif
     *  \note This property is not supported by every device. Therefore always call the function \b mvIMPACT::acquire::Component::isValid
     *  to check if this property is available or not.
     */
    PropertyI infoIOStatesAtExposureStart;
    /// \brief An integer property \b (read-only) containing the state of all the IO lines at the end of the frame exposure as a device specific bit field.
    /**
     *  See \b mvIMPACT::acquire::Request::infoIOStatesAtExposureStart for details on how to interpret the data.
     *
     *  \note This property is not supported by every device. Therefore always call the function \b mvIMPACT::acquire::Component::isValid
     *  to check if this property is available or not.
     */
    PropertyI infoIOStatesAtExposureEnd;
    /// \brief A 64 bit integer property \b (read-only) containing the X-offset of the image in pixels as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkOffsetX;
    /// \brief A 64 bit integer property \b (read-only) containing the Y-offset of the image in pixels as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkOffsetY;
    /// \brief A 64 bit integer property \b (read-only) containing the width of the image in pixels as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkWidth;
    /// \brief A 64 bit integer property \b (read-only) containing the height of the image in pixels as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkHeight;
    /// \brief A 64 bit integer property \b (read-only) containing the pixel format of the image as returned in the chunk data attached to the image.
    /**
     *  This will \b NOT be a valid mvIMPACT acquire pixel format but e.g. a GigEVision pixel format or somethins else.
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkPixelFormat;
    /// \brief A 64 bit integer property \b (read-only) containing the minimum value of dynamic range of the image as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkDynamicRangeMin;
    /// \brief A 64 bit integer property \b (read-only) containing the maximum value of dynamic range of the image as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkDynamicRangeMax;
    /// \brief A floating point property \b (read-only) containing the exposure time used to capture the image as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyF chunkExposureTime;
    /// \brief A 64 bit integer property \b (read-only) containing the timestamp value of the internal frame start signal of the image as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkTimestamp;
    /// \brief A 64 bit integer property \b (read-only) containing the status of all I/O lines at the time of the internal frame start signal of the image as returned in the chunk data attached to the image.
    /**
     *  \note
     *  For additional information about the chunk data format please refer to \ref ImageAcquisition_section_chunk.
     */
    PropertyI64 chunkLineStatusAll;
    /// \brief An enumerated integer property \b (read-only) containing the memory mode used for this request.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TRequestImageMemoryMode.
     *
     *  \note This property will become writeable if this request is in configuration mode.
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure
     */
    PropertyIRequestImageMemoryMode imageMemoryMode;
    /// \brief An enumerated integer property \b (read-only) containing the pixel format of this image.
    /**
     *  This might be important, when the image data needs to be processed or stored in
     *  a file or maybe even if the image shall be displayed.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TImageBufferPixelFormat.
     */
    PropertyIImageBufferPixelFormat imagePixelFormat;
    /// \brief A pointer property \b (read-only) containing the start address of the image data.
    /**
     *  This address in connection with \b mvIMPACT::acquire::Request::imageSize is sufficient
     *  to copy the complete image without having any additional information about it.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  FunctionInterface& fi = getFunctionInterfaceFromSomewhere();
     *  int currentRequestNr = getCurrentRequestNumberFromSomewhere();
     *  const Request* pRequest = fi.getRequest( currentRequestNr )
     *  char* pTempBuf = new char[pRequest->imageSize.read()];
     *  memcpy( pTempBuf, pRequest->imageData.read(), pRequest->imageSize.read() );
     *  // free the memory once no longer needed...
     * \endcode
     *  \endif
     *
     *  \note
     *  It's not always necessary to copy the image data! Each image buffer is an
     *  integral part of the \b mvIMPACT::acquire::Request object returned to the user by a call to
     *  the corresponding 'waitFor' function offered by the interface.
     *  The data in this buffer remains valid until the user either
     *  unlocks the request buffer or closes the \b mvIMPACT::acquire::Device again.
     *
     *  \note
     *  By unlocking the \b mvIMPACT::acquire::Request, the user informs the driver that this \b mvIMPACT::acquire::Request
     *  and the image buffer belonging to that \b mvIMPACT::acquire::Request is no longer needed by the
     *  user. The driver then queues this \b mvIMPACT::acquire::Request for capturing image data into it once
     *  again. However, once a \b mvIMPACT::acquire::Request has been returned to the user, its image buffer
     *  can't be overwritten by the driver! Therefore the user can work with, modify, store or
     *  copy the data safely until he unlocks the \b mvIMPACT::acquire::Request again.
     *
     *  \note This property will become writeable if this request is in configuration mode.
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::Request::imageSize, \n
     *  \b mvIMPACT::acquire::Request::imageFooterSize
     */
    PropertyPtr imageData;
    /// \brief An integer property \b (read-only) containing the size (in bytes) of the whole image.
    /**
     *  This value in connection with \b mvIMPACT::acquire::Request::imageData is sufficient
     *  to copy the complete image without having any additional information about it.
     *
     *  \note This property will become writeable if this request is in configuration mode.
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::Request::imageData, \n
     *  \b mvIMPACT::acquire::Request::imageFooterSize
     */
    PropertyI imageSize;
    /// \brief A pointer property \b (read-only) containing the start address of the footer associated with this image.
    /**
     *  A footer may contain additional data like e.g. information about the image (e.g. the exposure time used for this image).
     *  If this property contains the value '0' no footer is attached to this image.
     *
     *  \note
     *  In general it is not necessary to access footer data. Sometime e.g. when the hardware already does some image pre-processing
     *  a footer containing additional information about the image data might be attached to the image. In this case some additional
     *  information about the structure of the footer might be needed as well.
     */
    PropertyPtr imageFooter;
    /// \brief An integer property \b (read-only) containing the size (in bytes) of the footer associated with this image.
    ///
    /// If this property contains the value '0' no footer is attached to this image.
    PropertyI imageFooterSize;
    /// \brief An integer property \b (read-only) containing the offset (in bytes) to the next pixel of the specified channel of this image.
    /**
     *  \note
     *  This property will store \a imageChannelCount values. These can be queried one by one using the \a index parameter of the function
     *  \b mvIMPACT::acquire::PropertyI::read() or in a single call by using the overloaded \a read function accepting a reference to a
     *  vector.
     */
    PropertyI imagePixelPitch;
    /// \brief An integer property \b (read-only) containing the number of channels this image consists of.
    /**
     *  For an RGB image this value e.g. would be 3. This value also defines how many parameters are stored by the properties
     *  \b mvIMPACT::acquire::Request::imagePixelPitch, \n
     *  \b mvIMPACT::acquire::Request::imageLinePitch, \n
     *  \b mvIMPACT::acquire::Request::imageChannelBitDepth, \n
     *  \b mvIMPACT::acquire::Request::imageChannelDesc and \n
     *  \b mvIMPACT::acquire::Request::imageChannelOffset.
     *
     *  If e.g. the channel count is 3 a call like pRequest->imageChannelOffset.read( 2 ) would return the channel offset of
     *  channel 3 (as calls to the read functions of properties are '0' based).
     */
    PropertyI imageChannelCount;
    ///  \brief An integer property \b (read-only) containing the offset (in bytes) to each channel belonging to the current image relative to the address
    /**  contained in \b mvIMPACT::acquire::Request::imageData.
    *
    *  \note
    *  This property will store \a imageChannelCount values. These can be queried one by one using the \a index parameter of the function
    *  \b mvIMPACT::acquire::PropertyI::read() or in a single call by using the overloaded \a read function accepting a reference to a
    *  vector.
    */
    PropertyI imageChannelOffset;
    /// \brief An integer property \b (read-only) containing the number of effective bits stored in each channel belonging to the current image.
    /**
     *  \note
     *  This property will store \a imageChannelCount values. These can be queried one by one using the \a index parameter of the function
     *  \b mvIMPACT::acquire::PropertyI::read() or in a single call by using the overloaded \a read function accepting a reference to a
     *  vector.
     */
    PropertyI imageChannelBitDepth;
    /// \brief An integer property \b (read-only) containing the offset (in bytes) to the next line of each channel belonging to the current image.
    /**
     *  \note
     *  This property will store \a imageChannelCount values. These can be queried one by one using the \a index parameter of the function
     *  \b mvIMPACT::acquire::PropertyI::read() or in a single call by using the overloaded \a read function accepting a reference to a
     *  vector.
     */
    PropertyI imageLinePitch;
    /// \brief A string property \b (read-only) containing the string descriptors of each channel belonging to the current image.
    /**
     *  \note
     *  This property will store \a imageChannelCount values. These can be queried one by one using the \a index parameter of the function
     *  \b mvIMPACT::acquire::PropertyI::read() or in a single call by using the overloaded \a read function accepting a reference to a
     *  vector.
     *
     *  For an RGB image this property e.g. might contain three values "R", "G" and "B".
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  FunctionInterface& fi = getFunctionInterfaceFromSomewhere();
     *  int currentRequestNr = getCurrentRequestNumberFromSomewhere();
     *  const Request* pRequest = fi.getRequest( currentRequestNr )
     *  int channelCount = pRequest->imageChannelCount.read();
     *  for( int i=0; i<channelCount; i++ )
     *  {
     *    cout << "channel[" << i << "]: " << pRequest->imageChannelDesc.read( i ) << endl;
     *  }
     * \endcode
     *  \endif
     */
    PropertyS imageChannelDesc;
    /// \brief An integer property \b (read-only) containing the number of bytes per pixel in this image.
    PropertyI imageBytesPerPixel;
    /// \brief An integer property \b (read-only) containing the X-offset of the image in pixels.
    /**
     *  \note
     *  This feature will be supported by devices using a mvIMPACT Acquire driver greater or equal version 1.10.92.
     */
    PropertyI imageOffsetX;
    /// \brief An integer property \b (read-only) containing the Y-offset of the image in pixels.
    /**
     *  \note
     *  This feature will be supported by devices using a mvIMPACT Acquire driver greater or equal version 1.10.92.
     */
    PropertyI imageOffsetY;
    /// \brief An integer property \b (read-only) containing the width of the image in pixels.
    PropertyI imageWidth;
    /// \brief An integer property \b (read-only) containing the total width of the image in pixels if this buffer is part of a larger image.
    /**
     *  \note
     *  This feature will be supported by devices using a mvIMPACT Acquire driver greater or equal version 1.11.50. Not
     *  every device driver will be capable of dealing with multiple buffers forming a single image. If a device driver can
     *  not deliver parts of a larger image, then this property will always contain the same value as returned by
     *  \b mvIMPACT::acquire::Request::imageWidth.
     */
    PropertyI imageWidthTotal;
    /// \brief An integer property \b (read-only) containing the height of the image in pixels.
    PropertyI imageHeight;
    /// \brief An integer property \b (read-only) containing the total height of the image in pixels.
    /**
     *  \note
     *  This feature will be supported by devices using a mvIMPACT Acquire driver greater or equal version 1.11.50. Not
     *  every device driver will be capable of dealing with multiple buffers forming a single image. If a device driver can
     *  not deliver parts of a larger image, then this property will always contain the same value as returned by
     *  \b mvIMPACT::acquire::Request::imageHeight.
     */
    PropertyI imageHeightTotal;
    /// \brief An enumerated integer property \b (read-only) containing the bayer parity of this image if this buffer is part of a larger image.
    /**
     *  If the current image does not contain bayer data, this value will be \b mvIMPACT::acquire::bmpUndefined.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBayerMosaicParity.
     */
    PropertyIBayerMosaicParity imageBayerMosaicParity;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIRequestResult getRequestResult( void ) const
    {
        return requestResult;
    }
    PropertyIRequestState getRequestState( void ) const
    {
        return requestState;
    }
    PropertyI64 getInfoFrameID( void ) const
    {
        return infoFrameID;
    }
    PropertyI64 getInfoFrameNr( void ) const
    {
        return infoFrameNr;
    }
    PropertyI getInfoExposeStart_us( void ) const
    {
        return infoExposeStart_us;
    }
    PropertyI getInfoExposeTime_us( void ) const
    {
        return infoExposeTime_us;
    }
    PropertyI getInfoTransferDelay_us( void ) const
    {
        return infoTransferDelay_us;
    }
    PropertyF getInfoGain_dB( void ) const
    {
        return infoGain_dB;
    }
    PropertyI64 getInfoTimeStamp_us( void ) const
    {
        return infoTimeStamp_us;
    }
    PropertyF getInfoImageAverage( void ) const
    {
        return infoImageAverage;
    }
    PropertyI getInfoVideoChannel( void ) const
    {
        return infoVideoChannel;
    }
    PropertyICameraOutput getInfoCameraOutputUsed( void ) const
    {
        return infoCameraOutputUsed;
    }
    PropertyI getInfoLineCounter( void ) const
    {
        return infoLineCounter;
    }
    PropertyF getInfoMissingData_pc( void ) const
    {
        return infoMissingData_pc;
    }
    PropertyI getInfoIOStatesAtExposureStart( void ) const
    {
        return infoIOStatesAtExposureStart;
    }
    PropertyI getInfoIOStatesAtExposureEnd( void ) const
    {
        return infoIOStatesAtExposureEnd;
    }
    PropertyI64 getChunkOffsetX( void ) const
    {
        return chunkOffsetX;
    }
    PropertyI64 getChunkOffsetY( void ) const
    {
        return chunkOffsetY;
    }
    PropertyI64 getChunkWidth( void ) const
    {
        return chunkWidth;
    }
    PropertyI64 getChunkHeight( void ) const
    {
        return chunkHeight;
    }
    PropertyI64 getChunkPixelFormat( void ) const
    {
        return chunkPixelFormat;
    }
    PropertyI64 getChunkDynamicRangeMin( void ) const
    {
        return chunkDynamicRangeMin;
    }
    PropertyI64 getChunkDynamicRangeMax( void ) const
    {
        return chunkDynamicRangeMax;
    }
    PropertyI64 getChunkExposureTime( void ) const
    {
        return chunkExposureTime;
    }
    PropertyI64 getChunkTimestamp( void ) const
    {
        return chunkTimestamp;
    }
    PropertyI64 getChunkLineStatusAll( void ) const
    {
        return chunkLineStatusAll;
    }
    PropertyIRequestImageMemoryMode getImageMemoryMode( void ) const
    {
        return imageMemoryMode;
    }
    PropertyIImageBufferPixelFormat getImagePixelFormat( void ) const
    {
        return imagePixelFormat;
    }
    //PropertyPtr getImageData( void ) const { return imageData; }
    PropertyI getImageSize( void ) const
    {
        return imageSize;
    }
    PropertyI getImageFooterSize( void ) const
    {
        return imageFooterSize;
    }
    PropertyI getImagePixelPitch( void ) const
    {
        return imagePixelPitch;
    }
    PropertyI getImageChannelCount( void ) const
    {
        return imageChannelCount;
    }
    PropertyI getImageChannelOffset( void ) const
    {
        return imageChannelOffset;
    }
    PropertyI getImageChannelBitDepth( void ) const
    {
        return imageChannelBitDepth;
    }
    PropertyI getImageLinePitch( void ) const
    {
        return imageLinePitch;
    }
    PropertyS getImageChannelDesc( void ) const
    {
        return imageChannelDesc;
    }
    PropertyI getImageBytesPerPixel( void ) const
    {
        return imageBytesPerPixel;
    }
    PropertyI getImageOffsetX( void ) const
    {
        return imageOffsetX;
    }
    PropertyI getImageOffsetY( void ) const
    {
        return imageOffsetY;
    }
    PropertyI getImageWidth( void ) const
    {
        return imageWidth;
    }
    PropertyI getImageWidthTotal( void ) const
    {
        return imageWidthTotal;
    }
    PropertyI getImageHeight( void ) const
    {
        return imageHeight;
    }
    PropertyI getImageHeightTotal( void ) const
    {
        return imageHeightTotal;
    }
    PropertyIBayerMosaicParity getImageBayerMosaicParity( void ) const
    {
        return imageBayerMosaicParity;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class for sets of properties that can be modified by the user.
class ComponentCollection
//-----------------------------------------------------------------------------
{
protected:
    HOBJ m_hRoot;
    explicit ComponentCollection() : m_hRoot( INVALID_ID ) {}
    explicit ComponentCollection( HOBJ hRoot ) : m_hRoot( hRoot ) {}
    explicit ComponentCollection( Device* pDev ) : m_hRoot( INVALID_ID )
    {
        if( !pDev->isOpen() )
        {
            pDev->open();
        }
    }
public:
    virtual ~ComponentCollection() {}
    /// \brief Returns a unique identifier for the component collection referenced by this object.
    /**
     *  This handle will always reference an object of type \b mvIMPACT::acquire::ComponentList.
     *  \return A unique identifier for the component referenced by this object.
     */
    HOBJ hObj( void ) const
    {
        return m_hRoot;
    }
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    /// \brief Restores the default for every component of this collection.
    /**
     *  Calling this function will restore the default value for every component
     *  belonging to this collection.
     *
     *  \note The caller must have the right to modify the component. Otherwise
     *  an exception will be thrown.
     */
    void
#else
    /// \brief Restores the default for every component of this collection.
    /**
     *  Calling this function will restore the default value for every component
     *  belonging to this collection.
     *
     *  \note The caller must have the right to modify the component. Otherwise
     *  an exception will be thrown.
     *
     *  \return A const reference to the component.
     */
    const ComponentCollection&
#endif
    restoreDefault( void ) const
    {
        TPROPHANDLING_ERROR result;
        if( ( result = OBJ_RestoreDefault( m_hRoot ) ) != PROPHANDLING_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, m_hRoot );
        }
#ifndef DOTNET_ONLY_CODE
        return *this;
#endif
    }
};

//-----------------------------------------------------------------------------
/// \brief A helper class to control the way an image request will be processed.
class ImageRequestControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageRequestControl object.
    explicit ImageRequestControl(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a
        /// \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal image request control to access with
        /// this instance. A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableImageRequestControls,
        /// new image request controls can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createImageRequestControl
        const std::string& name = "Base" ) : ComponentCollection( pDev ), mode(),
        imageCount(), setting(), resultQueue(), requestToUse()
    {
        DeviceComponentLocator locator( pDev, dltRequestCtrl, name );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( mode, "Mode" );
        locator.bindComponent( imageCount, "ImageCount" );
        locator.bindComponent( setting, "Setting" );
        locator.bindComponent( resultQueue, "ResultQueue" );
        locator.bindComponent( requestToUse, "RequestToUse" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief The mode this object shall be operated in.
    /**
     *  Valid values for this property are defined by the enumeration
     *  \b mvIMPACT::acquire::TImageRequestControlMode.
     *
     *  This property can e.g. be used to prepare internal data structures without requesting a
     *  'real' image from the hardware. This can be useful when the image dimensions must be known
     *  before the first image is captured.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  Device* pDev = getDevicePointerFromSomewhere();
     *  ImageRequestControl irc(pDev);
     *  irc.mode.write( ircmTrial ); // set to prepare mode
     *  FunctionInterface fi(pDev);
     *  fi.imageRequestSingle( &irc ); // request dummy image
     *  int reqNr = fi.imageRequestWaitFor( 500 );
     *  // waitFor will return as fast as possible. No 'real' image will be taken
     *  // but a request object that contains a dummy image with the format, dimensions
     *  // and other information will be returned that is (apart from the pixel data)
     *  // similar to any 'real' image that would be captured with the current settings
     * \endcode
     *  \endif
     */
    PropertyIImageRequestControlMode mode;
    /// \brief An integer property defining the number of images to capture with each request.
    /**
     *  \note This property will currently have no effect.
     */
    PropertyI imageCount;
    /// \brief An integer property defining which setting will be used for the acquisition.
    /**
     *  This property defines a translation dictionary. It maps the name of the setting (the parameter
     *  passed to the function \b mvIMPACT::acquire::FunctionInterface::createSetting) to the actual
     *  underlying handle of the request. Therefore either the name or the handle of the setting can be used
     *  to set this property to the desired value.
     */
    PropertyI setting;
    /// \brief An integer property defining to which result queue the resulting image will be sent after the acquisition.
    PropertyI resultQueue;
    /// \brief An integer property defining which request object shall be used for the next image request.
    /**
     *  This is an advanced feature that should only be used if needed. A value smaller than 0 will
     *  result in an automatic mode. This is the default behaviour and the driver will decide which buffer to use next then.
     *
     *  A request can only be sent to the driver once. Until this request is returned to the user it can't
     *  be sent again thus this property must be changed for each request if multiple requests shall be sent to
     *  the capture queue.
     */
    PropertyI requestToUse;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIImageRequestControlMode    getMode( void ) const
    {
        return mode;
    }
    PropertyI                           getImageCount( void ) const
    {
        return imageCount;
    }
    PropertyI                           getSetting( void ) const
    {
        return setting;
    }
    PropertyI                           getResultQueue( void ) const
    {
        return resultQueue;
    }
    PropertyI                           getRequestToUse( void ) const
    {
        return requestToUse;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A default request factory.
/**
 *  Applications need to derive from this class and must re-implement the function
 *  \b mvIMPACT::acquire::RequestFactory::createRequest to work with custom
 *  objects derived from \b mvIMPACT::acquire::Request.
 *
 *  Deriving from \b mvIMPACT::acquire::Request can be useful when a certain
 *  device driver or device offers a custom feature that is returned as part of
 *  the request object that can not be accessed using the \b mvIMPACT::acquire::Request
 *  class offered by this interface.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  \b EXAMPLE
 *
 *  This shows how a request factory could be used to create custom request objects
 *  from within a \b mvIMPACT::acquire::FunctionInterface instance.
 *
 * \code
 *  class MyRequestFactory;
 *
 *  //-----------------------------------------------------------------------------
 *  // Example for a derived request object. It doesn't introduce new functionality
 *  // but rebinds an existing property. Custom properties could bound in a similar
 *  // way.
 *  class MyRequest : public mvIMPACT::acquire::Request
 *  //-----------------------------------------------------------------------------
 *  {
 *    friend class MyRequestFactory;
 *    void init( void )
 *    {
 *      DeviceComponentLocator locator(getComponentLocator());
 *      locator.bindComponent( myRequestResult, "Result" );
 *    }
 *  protected:
 *    explicit MyRequest( Device* pDev, int requestNr ) : Request(pDev, requestNr), myRequestResult()
 *    {
 *      init();
 *    }
 *  public:
 *    explicit MyRequest( const MyRequest& src ) : Request(src), myRequestResult(src.myRequestResult) {}
 *    MyRequest& operator=( const MyRequest& rhs )
 *    {
 *      if( this != &rhs )
 *      {
 *        Request::operator=( rhs );
 *        init();
 *      }
 *      return *this;
 *    }
 *    PropertyIRequestResult myRequestResult;
 *  };
 *
 *  //-----------------------------------------------------------------------------
 *  // Example for a factory that creates 'MyRequest' instances
 *  class MyRequestFactory : public mvIMPACT::acquire::RequestFactory
 *  //-----------------------------------------------------------------------------
 *  {
 *  public:
 *    virtual mvIMPACT::acquire::Request* createRequest( Device* pDev, int requestNr ) { return new MyRequest(pDev, requestNr); }
 *  };
 *
 * \endcode
 *
 *  Now the request factory must be passed to the constructor of the function interface
 *
 * \code
 *  //-----------------------------------------------------------------------------
 *  void fn( mvIMPACT::acquire::Device* pDev )
 *  //-----------------------------------------------------------------------------
 *  {
 *    // ... some code ...
 *    MyRequestFactory mrf;
 *    FunctionInterface fi(pDev, &mrf);
 *    // ... more additional code
 *    // assuming we got back a request from the driver at this point:
 *    const Request* pRequest = fi.getRequest( getRequestNrFromSomewhere() );
 *    if( pRequest->isOK() )
 *    {
 *      const MyRequest* pMyRequest(dynamic_cast<const MyRequest*>(pRequest));
 *      cout << pMyRequest->myRequestResult.name() << ": " << pMyRequest->myRequestResult.readS() << endl;
 *    }
 *    // ... probably even more additional code
 *  }
 * \endcode
 *  \endif
 */
class RequestFactory
//-----------------------------------------------------------------------------
{
public:
#if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
    virtual Request* createRequest(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::>DeviceManager object.
        Device* pDev,
        /// [in] The request number that that internally has been assigned to this
        /// request by the device driver.
        int requestNr )
    {
        return 0;
    }
#else
    virtual Request* createRequest(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::>DeviceManager object.
        Device* /*pDev*/,
        /// [in] The request number that that internally has been assigned to this
        /// request by the device driver.
        int /*requestNr*/ )
    {
        return 0;
    }
#endif // #if defined(WRAP_DOTNET) || defined(DOTNET_ONLY_CODE)
};

//-----------------------------------------------------------------------------
/// \brief The function interface to devices supported by this interface
/**
 *  This class contains all the basic functions needed when working with a device.
 *  There are not too many functions as most data will be represented by 'properties'
 *  in other classes of this module, keeping the set of functions to remember rather
 *  small. To construct a new function interface, a pointer to a \b mvIMPACT::acquire::Device object
 *  must be passed to the constructor of \b mvIMPACT::acquire::FunctionInterface. In order to work
 *  with the \b mvIMPACT::acquire::FunctionInterface object the device needs to be initialized, so
 *  if the \b mvIMPACT::acquire::Device object pointed to by \a pDev in the constructors parameter list
 *  hasn't been opened already the constructor will try to do that.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  \b EXAMPLE
 *
 *  This sample assumes a valid pointer to a \b mvIMPACT::acquire::Device object has been obtained.
 *  To find out how to do that see the detailed description of the class \b mvIMPACT::acquire::DeviceManager.
 *
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *
 *  using namespace std;
 *  using namespace mvIMPACT::acquire;
 *
 *  static volatile bool g_boTerminated = false;
 *
 *  //-----------------------------------------------------------------------------
 *  void showPropVal( const Property& prop )
 *  //-----------------------------------------------------------------------------
 *  {
 *    cout << prop.name() << ": " << prop.readS() << endl;
 *  }
 *
 *  //-----------------------------------------------------------------------------
 *  void captureThread( Device* pDev )
 *  //-----------------------------------------------------------------------------
 *  {
 *    if( !pDev )
 *    {
 *      return; // error! Invalid pointer!
 *    }
 *    FunctionInterface fi(pDev);
 *    SettingsBlueFOX setting(pDev);
 *
 *    int maxExposureTime = setting.cameraSetting.expose_us.read( Property::maxValue );
 *    // make sure enough requests are available:
 *    SystemSettings ss(pDev);
 *    const int REQUEST_COUNT = ss.requestCount.read();
 *    // prefill the default capture queue
 *    for( unsigned int i=0; i<REQUEST_COUNT; i++ )
 *    {
 *      int result = fi.imageRequestSingle();
 *      if( result != DMR_NO_ERROR )
 *      {
 *        cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
 *      }
 *    }
 *
 *    // run thread loop
 *    const Request* pRequest = 0;
 *    int requestNr = INVALID_ID;
 *    while( !g_boTerminated )
 *    {
 *      // wait for results from the default capture queue
 *      requestNr = fi.imageRequestWaitFor( 500 );
 *      if( fi.isRequestNrValid( requestNr ) )
 *      {
 *        pRequest = fi.getRequest( requestNr );
 *        if( pRequest->isOK() )
 *        {
 *          ++cnt;
 *          if( cnt%100 == 0 )
 *          {
 *            // modify the exposure time to see how modifying
 *            // properties affects the capture results
 *            setting.cameraSetting.expose_us.write( ( cnt * 100 ) % maxExposureTime );
 *            // display some statistics
 *            showPropVal( setting.cameraSetting.expose_us );
 *            showPropVal( statistics.framesPerSecond );
 *            showPropVal( statistics.captureTime_s );
 *          }
 *          // do whatever is necessary to display/store or process the image
 *        }
 *        else
 *        {
 *          cout << "Error: " << pRequest->getParamS( irpResult ) << endl;
 *        }
 *        fi.imageRequestUnlock( requestNr );
 *        fi.imageRequestSingle();
 *      }
 *      else
 *      {
 *        // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
 *        // additional information under TDMR_ERROR in the interface reference
 *        cout << "imageRequestWaitFor failed (" << requestNr << ")"
 *             << ", timeout value too small?" << endl;
 *      }
 *    }
 *
 *    // free resources
 *    fi.imageRequestReset( 0, 0 );
 *  }
 * \endcode
 *  \endif
 */
class FunctionInterface
//-----------------------------------------------------------------------------
{
    typedef std::vector<Request*> RequestVector;
#   ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        ComponentList                       m_requestList;
        ComponentList                       m_imageRequestControlList;
        mutable std::vector<std::string>    m_imageRequestControls;
        mutable RequestVector               m_requests;
        Request*                            m_pCurrentCaptureBufferLayout;
        mutable std::vector<std::string>    m_settings;
        mutable unsigned int                m_lastImageRequestChangedCounter;
        mutable unsigned int                m_lastSettingsListChangedCounter;
        Device*                             m_pDevice;
        RequestFactory*                     m_pRequestFactory;
        PropertyI                           m_requestCount;
        PropertyI                           m_captureBufferAlignment;
        ComponentList                       m_settingsList;
        unsigned int                        m_refCnt;
        ReferenceCountedData( Device* pDev, RequestFactory* pRequestFactory ) : m_requestList(), m_imageRequestControlList(),
            m_imageRequestControls(), m_requests(), m_pCurrentCaptureBufferLayout( 0 ), m_settings(), m_lastImageRequestChangedCounter( 0 ),
            m_lastSettingsListChangedCounter( 0 ), m_pDevice( pDev ), m_pRequestFactory( pRequestFactory ),
            m_requestCount(), m_captureBufferAlignment(), m_settingsList(), m_refCnt( 1 ) {}
        // make sure there are NO assignments or copy constructions by accident!
        ReferenceCountedData& operator=( const ReferenceCountedData& rhs );
        ReferenceCountedData( const ReferenceCountedData& src );
        ~ReferenceCountedData()
        {
            RequestVector::size_type rvSize = m_requests.size();
            for( RequestVector::size_type i = 0; i < rvSize; i++ )
            {
                delete m_requests[i];
            }
            delete m_pCurrentCaptureBufferLayout;
        }
    }* m_pRefData;
#   endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    Request* createRequest( int requestNr ) const
    //-----------------------------------------------------------------------------
    {
        if( m_pRefData->m_pRequestFactory )
        {
            return m_pRefData->m_pRequestFactory->createRequest( m_pRefData->m_pDevice, requestNr );
        }
        return new Request( m_pRefData->m_pDevice, requestNr );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    void updateRequests( void ) const
    //-----------------------------------------------------------------------------
    {
        RequestVector::size_type rListSize = static_cast<RequestVector::size_type>( m_pRefData->m_requestList.size() );
        RequestVector::size_type vSize = m_pRefData->m_requests.size();
        if( rListSize < vSize )
        {
            // requests have been deleted...
            for( RequestVector::size_type i = rListSize; i < vSize; i++ )
            {
                delete m_pRefData->m_requests[i];
            }
            m_pRefData->m_requests.resize( rListSize );
        }
        else if( rListSize > vSize )
        {
            // there are new requests
            ComponentIterator it( m_pRefData->m_requestList );
            it = it.firstChild();
            for( RequestVector::size_type i = 0; ( i < vSize ) && it.isValid(); i++ )
            {
                ++it;
            }
            while( it.isValid() )
            {
                // this might happen, if there are several instances referring to the same device
                // and another instance has created new requests in the meantime, or if this instance
                // has just been created
                m_pRefData->m_requests.push_back( createRequest( static_cast<int>( m_pRefData->m_requests.size() ) ) );
                ++it;
            }
        }
    }
    //-----------------------------------------------------------------------------
    void updateSettings( void ) const
    //-----------------------------------------------------------------------------
    {
        unsigned int actChangedCounter = m_pRefData->m_settingsList.changedCounter();
        if( m_pRefData->m_lastSettingsListChangedCounter != actChangedCounter )
        {
            // something has changed in the settings list -> update the string array
            // the string array must be filled from scratch each time because settings might have been
            // deleted as well...
            m_pRefData->m_settings.clear();
            ComponentIterator it( m_pRefData->m_settingsList );
            it = it.firstChild();
            while( it.isValid() )
            {
                // this might happen, if there are several instances referring to the same device
                // and another instance has created new settings in the meantime, or if this instance
                // has just been created
                m_pRefData->m_settings.push_back( it.name() );
                ++it;
            }
            m_pRefData->m_lastSettingsListChangedCounter = actChangedCounter;
        }
    }
    //-----------------------------------------------------------------------------
    void updateImageRequestControls( void ) const
    //-----------------------------------------------------------------------------
    {
        unsigned int actChangedCounter = m_pRefData->m_imageRequestControlList.changedCounter();
        if( m_pRefData->m_lastImageRequestChangedCounter != actChangedCounter )
        {
            // something has changed in the image request controls list -> update the string array
            // the string array must be filled from scratch each time because objects might have been
            // deleted as well...
            m_pRefData->m_imageRequestControls.clear();
            ComponentIterator it( m_pRefData->m_imageRequestControlList );
            it = it.firstChild();
            while( it.isValid() )
            {
                // this might happen, if there are several instances referring to the same device
                // and another instance has created new object in the meantime, or if this instance
                // has just been created
                m_pRefData->m_imageRequestControls.push_back( it.name() );
                ++it;
            }
            m_pRefData->m_lastImageRequestChangedCounter = actChangedCounter;
        }
    }
public:
    /// \brief Constructs a new function interface for the device pointed to by \a pDev.
    /**
     *  In order to work with the \b mvIMPACT::acquire::FunctionInterface object the device needs to be
     *  initialized, so if the \b mvIMPACT::acquire::Device object pointed to by \a pDev in the constructors
     *  parameter list hasn't been opened already the construtor will try to do that.
     *  Thus internally a call to \b mvIMPACT::acquire::Device::open might be preformed with all consequences
     *  following from this call.
     *
     *  \sa
     *  \b Device::open
     */
    explicit FunctionInterface(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] A pointer to a request factory.
        /// By supplying a custom request factory the user can control the type of request objects
        /// that will be created by the function interface.
        RequestFactory* pRequestFactory = 0 ) : m_pRefData( 0 )
    {
        if( !pDev->isOpen() )
        {
            pDev->open();
        }
        m_pRefData = new ReferenceCountedData( pDev, pRequestFactory );
        DeviceComponentLocator locator( pDev, dltSetting );
        m_pRefData->m_settingsList = ComponentList( locator.searchbase_id() );
        locator.bindSearchBaseList( pDev, dltRequest );
        m_pRefData->m_requestList = ComponentList( locator.searchbase_id() );
        locator.bindSearchBaseList( pDev, dltRequestCtrl );
        m_pRefData->m_imageRequestControlList = ComponentList( locator.searchbase_id() );
        locator.bindSearchBaseList( pDev, dltSystemSettings );
        locator.bindComponent( m_pRefData->m_requestCount, "RequestCount" );
        locator.bindSearchBaseList( pDev, dltInfo );
        locator.bindComponent( m_pRefData->m_captureBufferAlignment, "CaptureBufferAlignment" );
        updateRequests();
        locator.bindSearchBaseList( pDev, dltInfo );
        if( locator.findComponent( "CurrentRequestLayout" ) != INVALID_ID )
        {
            m_pRefData->m_pCurrentCaptureBufferLayout = createRequest( -1 );
        }
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::FunctionInterface from and exisiting one.
    explicit FunctionInterface(
        /// [in] A constant reference to the \b mvIMPACT::acquire::FunctionInterface object, this object shall be created from
        const FunctionInterface& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor.
    virtual ~FunctionInterface( void )
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::FunctionInterface objects
    FunctionInterface& operator=( const FunctionInterface& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Manually starts the acquisition engine of this device driver instance.
    /**
     *  Calling this function will manually start this device driver's acquisition engine. This will only
     *  have an effect on the overall behaviour if \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour is
     *  set to \b mvIMPACT::acquire::assbUser.
     *
     *  If supported by the device driver, starting and stopping the acquisition engine manually can sometimes
     *  help to overcome capture queue underruns or certain restrictions in the underlying device driver
     *  technology.
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::acquisitionStop, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - \b mvIMPACT::acquire::DMR_FEATURE_NOT_AVAILABLE if this device/driver combination does not support manual control over the acquisition engine. This will also happen if \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour is \b NOT set to \b mvIMPACT::acquire::assbUser.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int acquisitionStart( void ) const
    {
        return DMR_AcquisitionStart( m_pRefData->m_pDevice->hDrv() );
    }
    /// \brief Manually stops the acquisition engine of this device driver instance.
    /**
     *  Calling this function will manually stop this device drivers acquisition engine. This will only
     *  have effect on the overall behaviour, if \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour is
     *  set to \b mvIMPACT::acquire::assbUser.
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::acquisitionStart, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - \b mvIMPACT::acquire::DMR_FEATURE_NOT_AVAILABLE if this device/driver combination does not support manual control over the acquisition engine. This will also happen if \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour is \b NOT set to \b mvIMPACT::acquire::assbUser.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int acquisitionStop( void ) const
    {
        return DMR_AcquisitionStop( m_pRefData->m_pDevice->hDrv() );
    }
    /// \brief Creates a new \b mvIMPACT::acquire::ImageRequestControl object.
    /**
     *  This function creates a new \b mvIMPACT::acquire::ImageRequestControl based on an existing one. New \b mvIMPACT::acquire::ImageRequestControl instances can only be derived
     *  from \b mvIMPACT::acquire::ImageRequestControl instances that already exist. When the driver has been initialised there will at least be
     *  one base \b mvIMPACT::acquire::ImageRequestControl called 'Base', which acts as the base for all other request controls.
     *
     *  All \b mvIMPACT::acquire::ImageRequestControl constructed by the application must be derived either from this base or any of its children using this function.
     *
     *  When this function succeeds, the \b mvIMPACT::acquire::ImageRequestControl constructor can be called with the \a name parameter passed to this function
     *  to get access to this newly registered \b mvIMPACT::acquire::ImageRequestControl.
     *
     *  \note
     *  See all comments made under \b mvIMPACT::acquire::FunctionInterface::createSetting for understanding the relationship between base and
     *  derived \b mvIMPACT::acquire::ImageRequestControl objects.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int createImageRequestControl(
        /// [in] The name of the new \b mvIMPACT::acquire::ImageRequestControl object.
        const std::string& name,
        /// [in] The name of the \b mvIMPACT::acquire::ImageRequestControl the new object shall be derived from.
        const std::string& parent = "Base",
        /// [out] A pointer to a \b mvIMPACT::acquire::ComponentList object that will receive the ID of the newly created request control.
        /// This parameter can be 0 if the application is not interessted in that parameter.
        ComponentList* pNewRequestControl = 0 )
    {
        HLIST newID;
        int result = DMR_CreateRequestControl( m_pRefData->m_pDevice->hDrv(), name.c_str(), parent.c_str(), &newID, 0 );
        if( pNewRequestControl )
        {
            *pNewRequestControl = ComponentList( newID );
        }
        return result;
    }
    /// \brief Creates a new setting.
    /**
     *  This function creates a new setting base on an existing one. New settings can only be derived
     *  from settings that already exist. When the driver has been initialised there will at least be
     *  one base setting called 'Base', which acts as the base for all other settings.
     *
     *  When a new setting is created it derives all the properties from the parent setting. That means
     *  initially the new setting will contain the very same data as the parent setting. As long as a
     *  component hasn't been modified in the new setting it will depend on the parent settings data.
     *  That means if e.g. a property in the parent list is modified, the newly created setting will
     *  also benefit from the updated value.
     *
     *  To release a certain component from this dependency it must be assigned a new value. The function
     *  \b mvIMPACT::acquire::Component::isDefault will return false afterwards indicating that the component no longer
     *  depends on the parent.
     *
     *  To restore this parent <-> child dependency the user can call the function
     *  \b mvIMPACT::acquire::Component::restoreDefault. Afterwards settings applied to the parent component
     *  will also be visible in the child component again.
     *
     *  When a new setting has been created successfully the name used to create the setting can
     *  be passed to any of the constructors of the setting related classes (\b mvIMPACT::acquire::CameraSettingsBase,
     *  \b mvIMPACT::acquire::ImageProcessing, ...) to access the new components.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int createSetting(
        /// [in] The name of the setting to be created.
        const std::string& name,
        /// [in] The name of the setting to derive the new setting from.
        const std::string& parent = "Base",
        /// [out] A pointer to a \b mvIMPACT::acquire::ComponentList object that will receive the ID of the newly created setting.
        /// This parameter can be 0 if the application is not interessted in that parameter.
        ComponentList* pNewSetting = 0 )
    {
        HLIST newID = INVALID_ID;
        int result = DMR_CreateSetting( m_pRefData->m_pDevice->hDrv(), name.c_str(), parent.c_str(), &newID );
        updateSettings();
        if( pNewSetting )
        {
            if( newID == INVALID_ID )
            {
                *pNewSetting = ComponentList();
            }
            else
            {
                *pNewSetting = ComponentList( newID );
            }
        }
        return result;
    }
    /// \brief Returns the names of the settings available for this \b mvIMPACT::acquire::Device.
    /**
     *  This function returns a const reference to a string array containing the names of all settings available
     *  for the current \b mvIMPACT::acquire::Device. These names are valid constructor parameters for objects
     *  like \b mvIMPACT::acquire::CameraSettingsBase, \b mvIMPACT::acquire::ImageProcessing, \b mvIMPACT::acquire::ImageDestination or
     *  classes derived from these types.
     *
     *  New settings can be created by calling
     *  \b mvIMPACT::acquire::FunctionInterface::createSetting
     *  \return A const reference to an array containing the names of all request controls available.
     */
    const std::vector<std::string>& getAvailableSettings( void ) const
    {
        updateSettings();
        return m_pRefData->m_settings;
    }
    /// \brief Returns the names of the image request controls available for this \b mvIMPACT::acquire::Device.
    /**
     *  This function returns a const reference to a string array containing the names of all image request controls available
     *  for the current \b mvIMPACT::acquire::Device. These names are valid constructor parameters for objects
     *  of the type \b mvIMPACT::acquire::ImageRequestControl.
     *
     *  New image request controls can be created by calling \b mvIMPACT::acquire::FunctionInterface::createImageRequestControl.
     *  \return A const reference to an array containing the names of all settings available.
     */
    const std::vector<std::string>& getAvailableImageRequestControls( void ) const
    {
        updateImageRequestControls();
        return m_pRefData->m_imageRequestControls;
    }
    /// \brief Returns a pointer to the desired \b mvIMPACT::acquire::Request.
    /**
     *  This function returns a pointer to the \b mvIMPACT::acquire::Request stored at \a nr
     *  in the internal array of requests.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr refers to an invalid \b mvIMPACT::acquire::Request \n
     *  (e.g. when \a nr is higher than the actual number of \n
     *  available requests) a STL out_of_range exception will be thrown.
     *  \endif
     *  \return A pointer to a \b mvIMPACT::acquire::Request object.
     */
    Request* getRequest(
        /// [in] The number of the request to return
        int nr ) const
    {
        updateRequests();
        return m_pRefData->m_requests.at( nr );
    }
    /// \brief Returns a \b mvIMPACT::acquire::ComponentList object to a setting with a specified name.
    /**
     *  This function returns a \b mvIMPACT::acquire::ComponentList object to a setting with a specified name
     *  or will raise an exception if no such setting exists.
     *
     *  \return A \b mvIMPACT::acquire::ComponentList object to a setting with a specified name
     */
    ComponentList getSetting(
        /// [in] The name of the setting to locate
        const std::string& name ) const
    {
        ComponentIterator it( m_pRefData->m_settingsList );
        it = it.firstChild();
        while( it.isValid() )
        {
            if( name == it.name() )
            {
                return ComponentList( it );
            }
            ++it;
        }
        ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, PROPHANDLING_COMPONENT_NOT_FOUND, INVALID_ID, "Setting '" + name + "' could not be found" );
        return ComponentList();
    }
    /// \brief Allows to set a request into configuration mode.
    /**
     *  \note
     *  Another version of this function is available that might be nicer to use depending on personal
     *  preferences and use case: \b mvIMPACT::acquire::Request::configure.
     *
     *  In configuration mode certain properties like \b mvIMPACT::acquire::Request::imageData,
     *  \b mvIMPACT::acquire::Request::imageSize, \b mvIMPACT::acquire::Request::imageMemoryMode
     *  of a request object can be modified. This can be used to configure one or more requests
     *  to use a user supplied memory. To use only a subset of the \b mvIMPACT::acquire::Request
     *  objects available the \b mvIMPACT::acquire::ImageRequestControl::requestToUse feature
     *  can be used.
     *
     *  Only requests that are currently not used by the driver and are not locked
     *  because they contain image data that hasn't been processed can be set into
     *  configuration mode.
     *
     *  \note
     *  Instead of calling this function directly for most cases it is much more convenient to use
     *  the functions \b mvIMPACT::acquire::Request::attachUserBuffer and \b mvIMPACT::acquire::Request::detachUserBuffer
     *  instead.
     *
     *  User supplied buffers must follow the alignment and size requirements reported by versions of the
     *  function \b mvIMPACT::acquire::FunctionInterface::getCurrentCaptureBufferLayout. Calling
     *  a version of this function will return all the information required to allocate buffers that
     *  can be used to capture data for the specified settings.
     *
     *  When allocating memory on the heap, the complete buffer size is needed which is calculated like
     *  this:
     *
     *  \b size + alignment
     *
     *  \note
     *  The address passed to the request object must be aligned already!
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  Device* pDev = getDevicePointerFromSomewhere()
     *  FunctionInterface fi(pDev);
     *  Request* pRequest = fi.getRequest( 0 );
     *  // the buffer assigned to the request object must be aligned accordingly
     *  // the size of the user supplied buffer MUST NOT include the additional size
     *  // caused by the alignment
     *  if( pRequest->attachUserBuffer( getAlignedMemoryPtr(), getAlignedMemorySize() ) ) == DMR_NO_ERROR )
     *  {
     *    ImageRequestControl irc(pDev);
     *    irc.requestToUse.write( 0 ); // use the buffer just configured for the next image request
     *    // now the next image will be captured into the user supplied memory
     *    fi.imageRequestSingle( &irc ); // this will send request '0' to the driver
     *    // wait for the buffer. Once it has been returned by the driver AND the user buffer shall no
     *    // longer be used call
     *    if( pRequest->detachUserBuffer() != DMR_NO_ERROR )
     *    {
     *      // handle error
     *    }
     *    // now this request will use internal memory again.
     *  }
     *  else
     *  {
     *     // handle error
     *  }
     * \endcode
     *  \endif
     *  \note
     *  A request that is in configuration mode can't be sent to the driver for acquisition until
     *  \b mvIMPACT::acquire::Request::unlock or \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock has been called again.
     *  By using \b mvIMPACT::acquire::Request::attachUserBuffer and \b mvIMPACT::acquire::Request::detachUserBuffer this locking
     *  and unlocking is done internally thus the application does not need to worry about this.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::Request::attachUserBuffer, \n
     *  \b mvIMPACT::acquire::Request::detachUserBuffer, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestReset
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int imageRequestConfigure(
        /// [in] A const pointer to the request object to configure
        const Request* pRequest ) const
    {
        return DMR_ImageRequestConfigure( m_pRefData->m_pDevice->hDrv(), pRequest->getNumber(), 0, 0 );
    }
    /// \brief Deletes all requests currently queued for the specified \b mvIMPACT::acquire::ImageRequestControl
    /**
     *  This function will terminate all running image acquisitions associated with the queue
     *  bound to the specified image request control and in addition to that
     *  will empty the queue of pending image requests for that queue.
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int imageRequestReset(
        /// [in] The \b mvIMPACT::acquire::ImageRequestControl for which all
        /// the requests shall be cancelled.
        int requestCtrlNr,
        /// [in] Currently unsupported. MUST be set 0.
        int mode ) const
    {
        return DMR_ImageRequestReset( m_pRefData->m_pDevice->hDrv(), requestCtrlNr, mode );
    }
    /// \brief Sends an image request to the \b mvIMPACT::acquire::Device driver.
    /**
     *  This functions sends a single image request to the capture device. To wait for the image
     *  to become ready call the function
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor.
     *
     *  \attention
     *  In order to make sure that no image data is lost, it is important to understand how
     *  the request mechanism works. The driver works with a fixed number of \b mvIMPACT::acquire::Request
     *  objects. The number of \b mvIMPACT::acquire::Request objects available to the driver can be set
     *  by modifying the property \b mvIMPACT::acquire::SystemBase::requestCount. Each \b mvIMPACT::acquire::Request will
     *  consume a certain amount of memory. Once an image has been captured by the \b mvIMPACT::acquire::Request,
     *  this amount will be slightly more than the image itself needs in memory, so modify this
     *  parameter gently. On the other hand, it's necessary to have more than a single request
     *  in order to ensure a lossless acquisition. E.g. when working with free running cameras an
     *  image might be lost because while an image has been captured and is being processed, the next
     *  vertical sync. pulse is already missed before the \b mvIMPACT::acquire::Request is unlocked again.
     *
     *  \note
     *  When \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour is
     *  set to \b mvIMPACT::acquire::assbUser several calls to \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle
     *  will \b NOT start the data acquisition! After requests have been send down to the driver,
     *  \b mvIMPACT::acquire::FunctionInterface::acquisitionStart must be called. For performance reasons some device drivers
     *  will \b NOT allow to request data into buffers which have not been known to the driver when \b mvIMPACT::acquire::FunctionInterface::acquisitionStart
     *  was called, thus before starting the acquisition, \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle
     *  should be called as many times as there are request objects.
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::acquisitionStart, \n
     *  \b mvIMPACT::acquire::FunctionInterface::acquisitionStop, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestReset, \n
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor \n
     *  \b mvIMPACT::acquire::Request::configure, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - \b mvIMPACT::acquire::DEV_NO_FREE_REQUEST_AVAILABLE if all mvIMPACT::acquire::Request objects are currently in use (either owned by the user or queued for acquisition).
     *  - \b mvIMPACT::acquire::DEV_INVALID_REQUEST_NUMBER if the request number defined by the property \b ImageRequestControl::requestToUse is invalid (out of bounds).
     *  - \b mvIMPACT::acquire::DEV_REQUEST_ALREADY_IN_USE if the request number defined by the property \b ImageRequestControl::requestToUse is already in use (either owned by the user or queued for acquisition).
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int imageRequestSingle(
        /// [in] A pointer to the \b mvIMPACT::acquire::ImageRequestControl object
        /// to be used for this request. \b mvIMPACT::acquire::ImageRequestControl
        /// objects define (among other things), which setting will
        /// be used for this image acquisition (via the property
        /// \b mvIMPACT::acquire::ImageRequestControl::setting).
        ImageRequestControl* pImageRequestControl = 0,
        /// [out] If a valid pointer to an integer is passed here, this
        /// variable receives the number of the \b mvIMPACT::acquire::Request which
        /// will be used to process this request.
        int* pRequestUsed = 0 ) const
    {
        return DMR_ImageRequestSingle( m_pRefData->m_pDevice->hDrv(), pImageRequestControl ? pImageRequestControl->hObj() & 0xFFFF : 0, pRequestUsed );
    }
    /// \brief Unlocks the request for the driver again.
    /**
     *  \note
     *  Another version of this function is available that might be nicer to use depending on personal
     *  preferences and use case: \b mvIMPACT::acquire::Request::unlock. Unlocking a certain request
     *  might be nicer by using this function, e.g. unlocking \b ALL requests in a loop is probably easier
     *  done by writing
     *  \if DOXYGEN_CPP_DOCUMENTATION
     * \code
     *  Device* pDev = getDevicePointerFromSomewhere()
     *  FunctionInterface fi(pDev);
     *  const int rc(static_cast<int>(fi.requestCount()));
     *  for( int i=0; i<rc; i++ )
     *  {
     *    fi.imageRequestUnlock( i );
     *  }
     * \endcode
     *  \endif
     *
     *  To ensure that no image data is overwritten by another image request while the user still
     *  deals with the image from a previous acquisition each image buffer will be locked by the driver when
     *  it is returned to the user by a call to \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor.
     *  No new image will be captured into the same buffer until the user unlocks the buffer again
     *  by calling \b mvIMPACT::acquire::FunctionInterface::imageRequestUnlock.
     *
     *  \note
     *  After unlocking a request it is no longer guaranteed that the memory once referenced by the
     *  request and the image buffer belonging to it stays valid, so do \b NEVER try to access
     *  memory belonging to an unlocked request object. If you need to copy the image buffer or
     *  modify it in any other way, do everything you have to do \b BEFORE calling this function!
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestReset, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor, \n
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestConfigure, \n
     *  \b mvIMPACT::acquire::Request::configure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - \b mvIMPACT::acquire::DEV_REQUEST_CANT_BE_UNLOCKED if the request number defined by \a nr is currently not owned by the application (either already unlocked or queued for acquisition).
     *  - \b mvIMPACT::acquire::DMR_INPUT_BUFFER_TOO_SMALL if the request number defined by \a nr has been configured to capture into a user supplied buffer but the buffer is too small to acquire data into.
     *  - \b mvIMPACT::acquire::DEV_REQUEST_BUFFER_MISALIGNED if the request number defined by \a nr has been configured to capture into a user supplied buffer but this buffer is not aligned in order to the alignment restrictions reported by this driver/device combination.
     *  - \b mvIMPACT::acquire::DEV_REQUEST_BUFFER_INVALID if the request number defined by \a nr has been configured to capture into a user supplied buffer but an invalid (NULL) pointer has been attached to the request.
     *  - \b mvIMPACT::acquire::DEV_INVALID_REQUEST_NUMBER if the request number defined by \a nr is invalid (out of bounds).
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int imageRequestUnlock(
        /// [in] The number of the request to unlock. This is typically
        /// a value returned from a call to
        /// \b mvIMPACT::acquire::FunctionInterface::imageRequestWaitFor.
        int nr ) const
    {
        return DMR_ImageRequestUnlock( m_pRefData->m_pDevice->hDrv(), nr );
    }
    /// \brief Waits for an image request to become ready.
    /**
     *  This function waits for an image request previously sent to the capture device by calling
     *  \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle.
     *
     *  \note Whenever a \b mvIMPACT::acquire::Request is returned to the user the image data
     *  described by the \b mvIMPACT::acquire::Request remains valid until the user unlocks
     *  the image buffer again or the driver is closed.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Request::unlock, \n
     *     mvIMPACT::acquire::FunctionInterface::imageRequestUnlock, \n
     *     mvIMPACT::acquire::FunctionInterface::getRequest
     *  \return If the return value is positive, the return value is the request number of the
     *  image request in the interface's internal array of image requests. In this case the user
     *  can call \b mvIMPACT::acquire::FunctionInterface::getRequest to get access to the
     *  results. \n \n If the result is negative, the return value is an error code.
     *  The error code that is returned in most cases will be \b mvIMPACT::acquire::DEV_WAIT_FOR_REQUEST_FAILED.
     *  To find out possible reasons for this error, look at the corresponding explanation.
     */
    int imageRequestWaitFor(
        /// [in] The maximum wait time in milliseconds (ms) for
        /// this request to become ready. If \a timeout_ms
        /// is '-1', the function's time-out interval never
        /// elapses. Please note that each request has its own timeout that is independent from
        /// this wait timeout, thus this function will return with a valid request after the timeout
        /// for this request has elapsed even if e.g. a trigger has not been detected. For detailed
        /// information on the interaction of the timeout of this function and the timeout of a
        /// request please refer to the chapter \ref ImageAcquisition_section_capture in the C++ section.
        int timeout_ms,
        /// [in] The queue where to wait for the request.
        /// The number of request queues available depends on the number of video channels offered by the
        /// device. The queue a processed request ends up in can be defined using the property
        /// \b mvIMPACT::acquire::ImageRequestControl::resultQueue.
        int queueNr = 0 ) const
    {
        int requestNr;
        TDMR_ERROR result = DMR_ImageRequestWaitFor( m_pRefData->m_pDevice->hDrv(), timeout_ms, queueNr, &requestNr );
        return ( result != DMR_NO_ERROR ) ? static_cast<int>( result ) : requestNr;
    }
#ifndef WRAP_DOTNET
    /// \brief Returns information about the current capture buffer requirements.
    /**
     *  When an application wants to provide capture buffers, this function will be needed in order to
     *  get information on how the capture buffers must be constructed. The most important parameters
     *  will problably be \a alignment and \a size, thus for most application calling the other overload
     *  of this function will probably be enough.
     *
     *  To find out more about capturing to user memory please refer to the application
     *
     *  \attention
     *  This feature has been introduced in version 1.12.68 of this SDK. If this application has been compiled
     *  with at least this version but runs on systems with drivers old than that, this feature might not be
     *  available and drivers should be updated then.
     *
     *  \sa mvIMPACT::acquire::Request::configure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int getCurrentCaptureBufferLayout(
        /// [in] A reference to the \b mvIMPACT::acquire::ImageRequestControl object
        /// containing the setting for which the current capture buffer layout shall
        /// be queried. The setting can be defined via the property
        /// \b mvIMPACT::acquire::ImageRequestControl::setting.
        ImageRequestControl& imageRequestControl,
        /// [out] On a successful call this variable will receive a pointer to a
        /// \b mvIMPACT::acquire::Request object reflecting the current
        /// capture buffer layout.
        Request** ppRequest = 0,
        /// [out] On a successful call this variable will receive the alignment
        /// needed for capturing into user supplied buffers that use the
        /// current settings.
        int* pAlignment = 0 ) const
    {
        int result = DMR_NO_ERROR;
        if( ppRequest )
        {
            if( !m_pRefData->m_pCurrentCaptureBufferLayout )
            {
                // This feature is available for every driver, but could not be detected thus an
                // update might be required.
                return DMR_NEWER_LIBRARY_REQUIRED;
            }
            const TImageRequestControlMode previousMode( imageRequestControl.mode.read() );
            if( previousMode != ircmUpdateBufferLayout )
            {
                imageRequestControl.mode.write( ircmUpdateBufferLayout );
            }
            result = imageRequestSingle( &imageRequestControl );
            if( previousMode != ircmUpdateBufferLayout )
            {
                imageRequestControl.mode.write( previousMode );
            }
            *ppRequest = m_pRefData->m_pCurrentCaptureBufferLayout;
        }
        if( pAlignment )
        {
            *pAlignment = m_pRefData->m_captureBufferAlignment.read();
        }
        return result;
    }
#endif // #ifndef WRAP_DOTNET
    /// \brief Returns information about the current capture buffer requirements.
    /**
     *  When an application wants to provide capture buffers, this function will be needed in order to
     *  get information on how the capture buffers must be constructed.
     *
     *  To find out more about capturing to user memory please refer to the application
     *  \ref CaptureToUserMemory.cpp
     *
     *  \note
     *  This feature has been introduced in version 1.12.68 of this SDK. If this application has been compiled
     *  with at least this version but runs on systems with drivers old than that, this feature might not be
     *  available and drivers should be updated then.
     *
     *  \sa
     *  \b mvIMPACT::acquire::Request::configure
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int getCurrentCaptureBufferLayout(
        /// [in] A reference to the \b mvIMPACT::acquire::ImageRequestControl object
        /// containing the setting for which the current capture buffer layout shall
        /// be queried. The setting can be defined via the property
        /// \b mvIMPACT::acquire::ImageRequestControl::setting.
        ImageRequestControl& imageRequestControl,
        /// [out] On a successful call this variable will receive the size
        /// needed for capturing into user supplied buffers that use the
        /// current settings.
        int& size,
        /// [out] On a successful call this variable will receive the alignment
        /// needed for capturing into user supplied buffers that use the
        /// current settings.
        int& alignment ) const
    {
        Request* pRequest = 0;
        int result = getCurrentCaptureBufferLayout( imageRequestControl, &pRequest, &alignment );
        if( m_pRefData->m_pCurrentCaptureBufferLayout )
        {
            size = m_pRefData->m_pCurrentCaptureBufferLayout->imageSize.read() + m_pRefData->m_pCurrentCaptureBufferLayout->imageFooterSize.read();
        }
        else
        {
            size = 0;
        }
        return result;
    }
    /// \brief Check if \a nr specifies a valid \b mvIMPACT::acquire::Request
    /**
     *  \return \n
     *  - true if \a nr specifies a valid request.
     *  - false otherwise
     */
    bool isRequestNrValid( int nr ) const
    {
        return nr >= 0;
    }
    /// \brief Checks if a request has been processed successfully(\b deprecated).
    /**
     *  \deprecated
     *  This function has been declared deprecated and will be removed in future versions of this interface.
     *  Use \b mvIMPACT::acquire::Request::isOK() instead and see the corresponding 'Porting existing code'
     *  section in the documentation.
     *  \return
     *  - true if the request has been processed successfully.
     *  - false otherwise.
     */
    MVIMPACT_DEPRECATED_CPP( bool isRequestOK(
                                 /// [in] The request to check
                                 const Request* p ) const );
    /// \brief Loads a previously stored setting.
    /**
     *  This function can be used to restore a previously stored setting again.
     *
     *  To load a setting from a file the \b mvIMPACT::acquire::sfFile should be specified
     *  as part of \a storageflags. To load a setting from a platform specific location
     *  such as the Registry under Windows&copy; \b mvIMPACT::acquire::sfNative should be
     *  specified. It's not allowed to combine \b mvIMPACT::acquire::sfFile and
     *  \b mvIMPACT::acquire::sfNative for this operation.
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::saveSetting.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int loadSetting(
        /// [in] The name or the full path under where the setting is located
        const std::string& name,
        /// [in] The flags which define which information shall be read from the location
        /// and how this information shall be interpreted.
        TStorageFlag storageflags = sfNative,
        /// [in] Specifies where the information is located.
        TScope scope = sGlobal ) const
    {
        return DMR_LoadSetting( m_pRefData->m_pDevice->hDrv(), name.c_str(), storageflags, scope );
    }
    /// \brief Loads the default settings
    /**
     *  This function will try to load the settings from a default location. This function
     *  can only succeed if a setting has been stored previously by a call to
     *  \b mvIMPACT::acquire::FunctionInterface::saveSettingToDefault.
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int loadSettingFromDefault(
        /// [in] Specifies where the information is located.
        TScope scope = sUser ) const
    {
        return DMR_LoadSettingFromDefault( m_pRefData->m_pDevice->hDrv(), scope );
    }
    /// \brief Returns the number of available request objects.
    /**
     *  \return Returns the number of available request objects.
     */
    unsigned int requestCount( void ) const
    {
        updateRequests();
        return static_cast<unsigned int>( m_pRefData->m_requests.size() );
    }
    /// \brief Stores the current settings.
    /**
     *  This function can be used to store the current settings either in a XML-file or
     *  (under Windows&copy;) into the Registry. A setting contains all the values set
     *  for properties that control the overall way an image is acquired( e.g. the exposure time, etc.).
     *
     *  To store a setting in a file the \b mvIMPACT::acquire::sfFile should be specified
     *  as part of \a storageflags. To store a setting in a platform specific location
     *  such as the Registry under Windows&copy; \b mvIMPACT::acquire::sfNative should be
     *  specified. Both flags can be combined. In that case the same setting will be stored in a
     *  file \b AND in a platform specific location if these location differ (platform dependent!).
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int saveSetting(
        /// [in] The name or the full path under which this setting shall be stored
        const std::string& name,
        /// [in] The flags which define which information shall be stored and how this
        /// information shall be stored.
        TStorageFlag storageflags = sfNative,
        /// [in] Specifies where the information shall be stored.
        TScope scope = sGlobal ) const
    {
        return DMR_SaveSetting( m_pRefData->m_pDevice->hDrv(), name.c_str(), storageflags, scope );
    }
    /// \brief Stores the current settings under a default location.
    /**
     *  Under Windows&copy; this will be in the Registry. A setting contains all the values set
     *  for properties that control the overall way an image is acquired( e.g. the exposure time, etc.).
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int saveSettingToDefault(
        /// [in] Specifies where the information shall be stored.
        TScope scope = sUser ) const
    {
        return DMR_SaveSettingToDefault( m_pRefData->m_pDevice->hDrv(), scope );
    }
#   ifndef WRAP_PYTHON
    /// \deprecated
    /// \brief Stores the current system settings under a default location (\b deprecated).
    /**
     *  \note
     *  This function has been declared deprecated and will be removed in version 2.0.0 of this interface.
     *  All features that have been stored using this method will now also be
     *  handled by the functions that load and save settings.
     *  Please see the corresponding 'Porting existing code' section in the documentation.
     *
     *  These are the settings which can be altered by creating an instance of the class
     *  \b mvIMPACT::acquire::SystemBase. Under Windows&copy; this function will store the current settings
     *  in the Registry.
     *
     *  Modifying data in this section of properties will affect every device belonging to the same
     *  family (e.g. every USB 2.0 camera device) when the data is saved. When the modified data is
     *  stored it will be restored by each device belonging to the same family during the next
     *  initialisation. So at runtime changing and saving the settings of one device will not affect
     *  the behaviour of another until the data has been stored and the other device has been closed
     *  and reopened again.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    MVIMPACT_DEPRECATED_CPP( int saveSystemToDefault(
                                 /// [in] Specifies where the information shall be stored.
                                 TScope scope = sUser ) const );
#   endif // #  ifndef WRAP_PYTHON
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#   if !defined(WRAP_PYTHON) // To date, no customer uses Python, so we don't need backward compatibility here.
//-----------------------------------------------------------------------------
inline int FunctionInterface::saveSystemToDefault( TScope scope /* = sUser */ ) const
//-----------------------------------------------------------------------------
{
    return DMR_SaveSystemToDefault( m_pRefData->m_pDevice->hDrv(), scope );
}

//-----------------------------------------------------------------------------
inline bool FunctionInterface::isRequestOK( const Request* p ) const
//-----------------------------------------------------------------------------
{
    if( p )
    {
        return ( p->requestResult.read() == rrOK );
    }
    return false;
}
#   endif // #if !defined(WRAP_PYTHON)
#endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

//-----------------------------------------------------------------------------
/// \brief A base class to access various general information about the device and its driver
/**
 *  This class contains a collection of properties providing various information about
 *  the device and its driver.
 *
 *  Instances of this class can't be constructed directly. Use one of the derived types.
 */
class InfoBase : public ComponentCollection
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Constructs a new \b mvIMPACT::acquire::InfoBase object.
    explicit InfoBase(  /// A pointer to a \b mvIMPACT::acquire::Device object obtained from
        /// a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : ComponentCollection( pDev ), deviceDriverVersion(), driverDate(),
        driverVersion(), state(), loadedSettings(), logFile(), captureBufferAlignment(),
        recommendedListsForUIs()
    {
        DeviceComponentLocator locator( pDev, dltInfo );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( deviceDriverVersion, "DeviceDriverVersion" );
        locator.bindComponent( driverDate, "DriverDate" );
        locator.bindComponent( driverVersion, "DriverVersion" );
        locator.bindComponent( state, "State" );
        locator.bindComponent( loadedSettings, "LoadedSettings" );
        locator.bindComponent( logFile, "LogFile" );
        locator.bindComponent( captureBufferAlignment, "CaptureBufferAlignment" );
        locator.bindComponent( recommendedListsForUIs, "RecommendedListsForUIs" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property \b (read-only) containing the device driver version used by this device.
    /**
     *  This is the version of the underlying hardware driver. For device drivers that don't have another
     *  user mode driver below the interface driver this property will contain the same version information
     *  as the property \b mvIMPACT::acquire::InfoBase::driverVersion.
     */
    PropertyS deviceDriverVersion;
    /// \brief A string property \b (read-only) containing the date the device driver has been compiled.
    PropertyS driverDate;
    /// \brief A string property \b (read-only) containing the version number of the device driver.
    PropertyS driverVersion;
    /// \brief An enumerated integer property \b (read-only) containing the current state of this device.
    /**
     *  This property e.g. provides information about the current state of the device. For USB devices this can e.g. indicate whether
     *  a device is currently plugged into the system or not.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceState.
     *
     *  \b GenICam/GenTL \b device \b specific:
     *  In order to reduce the amount of network traffic to a minimum, this property will only be updated automatically
     *  for network devices if the property \b mvIMPACT::acquire::Device::registerErrorEvent is set to
     *  \b mvIMPACT::acquire::bTrue (which is the default behaviour). If the IP addresses stay the same the
     *  connection is automatically re-established then once the device is detected by the driver again.
     *  However if the IP address of the device and/or the network adapter of the system it is used from changes
     *  \b mvIMPACT::acquire::DeviceManager::updateDeviceList()
     *  must be called regardless of the value of \b mvIMPACT::acquire::Device::registerErrorEvent before a device
     *  that was lost can re-establish a connection to the capture driver.
     */
    PropertyIDeviceState state;
    /// \brief A string property \b (read-only) containing the name of the setting currently loaded.
    PropertyS loadedSettings;
    /// \brief A string property \b (read-only) containing the name and the full path of the current log-file for this device.
    PropertyS logFile;
    /// \brief An integer property \b (read-only) containing the capture buffer alignment in bytes needed by this device driver.
    PropertyI captureBufferAlignment;
    /// \brief A string property \b (read-only) containing an array of full search pathes to lists which are recommended to be displayed in a user interface that is created dynamically.
    PropertyS recommendedListsForUIs;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyS getDeviceDriverVersion( void ) const
    {
        return deviceDriverVersion;
    }
    PropertyS getDriverDate( void ) const
    {
        return driverDate;
    }
    PropertyS getDriverVersion( void ) const
    {
        return driverVersion;
    }
    PropertyIDeviceState getState( void ) const
    {
        return state;
    }
    PropertyS getLoadedSettings( void ) const
    {
        return loadedSettings;
    }
    PropertyS getLogFile( void ) const
    {
        return logFile;
    }
    PropertyI getCaptureBufferAlignment( void ) const
    {
        return captureBufferAlignment;
    }
    PropertyS getRecommendedListsForUIs( void ) const
    {
        return recommendedListsForUIs;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to query various general information about the device, its driver and other information.
class Info : public InfoBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::Info object.
    explicit Info(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : InfoBase( pDev ) {}
};

//-----------------------------------------------------------------------------
/// \brief A base class for essential device related settings.
/**
 *  This class acts as a base class for essential device related settings. It only contains
 *  features that are available for every device.
 */
class BasicDeviceSettings : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::BasicDeviceSettings object.
    explicit BasicDeviceSettings(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : ComponentCollection( pDev ), basedOn(), imageRequestTimeout_ms()
    {
        DeviceComponentLocator locator( pDev, dltSetting, settingName );
        locator.bindComponent( basedOn, "BasedOn" );
        locator.bindSearchBase( locator.searchbase_id(), "Camera" );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( imageRequestTimeout_ms, "ImageRequestTimeout_ms" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A string property \b (read-only) containing the name of the setting this setting is based on.
    PropertyS basedOn;
    /// \brief An integer property defining the maximum time to wait for an image in ms.
    /**
     *  When this property is set to 0 the timeout never elapses. If the timeout this property
     *  defines elapses the currently processed request will be returned to the user even if no data
     *  has been captured so far. The result structure of the request object should be examined in that
     *  case. Also this should be considered as an error from the users point of view in which case he
     *  should clean up the acquisition queue by calling \b mvIMPACT::acquire::FunctionInterface::imageRequestReset.
     *  Afterwards the capture loop can be restarted.
     */
    PropertyI imageRequestTimeout_ms;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyS getBasedOn( void ) const
    {
        return basedOn;
    }
    PropertyI getImageRequestTimeout_ms( void ) const
    {
        return imageRequestTimeout_ms;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to configure the behaviour of trigger signals
/**
 *  Features in this class will open up a more flexible way to configure certain
 *  trigger events for devices then the previous features('triggerInterface',
 *  'triggerSource' and 'triggerMode' used.
 *
 *  \note
 *  Not ever device will support every property listed in this class for every trigger
 *  mode. Always call \b mvIMPACT::acquire::Component::isValid to check wether
 *  a feature is available or not before using it. Otherwise an exception will
 *  be generated.
 */
class TriggerControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class CameraSettingsBlueCOUGAR;
    friend class CameraSettingsFrameGrabber;
    Method m_triggerSoftware;
    explicit TriggerControl( HLIST hList ) : ComponentCollection( hList ), m_triggerSoftware()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( m_triggerSoftware, "TriggerSoftware@i" );
        locator.bindComponent( triggerMode, "TriggerMode" );
        locator.bindComponent( triggerSource, "TriggerSource" );
        locator.bindComponent( triggerActivation, "TriggerActivation" );
        locator.bindComponent( triggerDivider, "TriggerDivider" );
        // unfortunately there are a couple of different names around...
        locator.bindComponent( triggerDelayAbs_us, "TriggerDelayAbs_us" );
        if( !triggerDelayAbs_us.isValid() )
        {
            locator.bindComponent( triggerDelayAbs_us, "TriggerDelayAbs" );
            if( !triggerDelayAbs_us.isValid() )
            {
                locator.bindComponent( triggerDelayAbs_us, "TriggerDelay" );
            }
        }
        locator.bindComponent( triggerDelayLines, "TriggerDelayLines" );
        locator.bindComponent( triggerOverlap, "TriggerOverlap" );
    }
public:
    /// \brief Returns a name of the trigger being configured.
    std::string getDescription( void ) const
    {
        return ComponentList( m_hRoot ).name();
    }
    /// \brief Generates of software trigger command.
    /**
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int triggerSoftware( void ) const
    {
        if( !m_triggerSoftware.isValid() )
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
        return m_triggerSoftware.call();
    }
    PYTHON_ONLY( %immutable; )
    /// \brief Defines if the selected trigger is active.
    PropertyI64 triggerMode;
    /// \brief Defines the signal that will cause the trigger event.
    /**
     *  Currently this property will define a translation dictionary that can e.g. be queried via a call to
     *  \b mvIMPACT::acquire::PropertyI64::getTranslationDict. The resulting translation dictionary can be used
     *  to select the desired trigger source either via a string or the associated 64 bit integer value.
     */
    PropertyI64 triggerSource;
    /// \brief An enumerated 64 bit integer property that defines the start condition for the selected trigger.
    /**
     *  Valid value will be 'RisingEdge', 'FallingEdge', 'AnyEdge', 'LevelLow' or 'LevelHigh'.
     */
    PropertyI64 triggerActivation;
    /// \brief An integer property used to divide the number of incoming trigger pulses by an integer factor.
    /**
     *  E.g. setting this value to 2 would generate 5 internal trigger events from a 10 Hz external signal.
     */
    PropertyI triggerDivider;
    /// \brief Specifies the absolute delay in microseconds (us) to apply after the trigger reception before
    /// effectively activating it.
    PropertyF triggerDelayAbs_us;
    /// \brief Specifies the absolute delay in lines to apply after the trigger reception before effectively activating it.
    PropertyI triggerDelayLines;
    /// \brief Specifies the type trigger overlap permitted with the previous frame.
    ///
    /** This defines when a valid trigger will be accepted (or latched) for a new frame.
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceTriggerOverlap.
     */
    PropertyI64DeviceTriggerOverlap triggerOverlap;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyI64 getTriggerMode( void ) const
    {
        return triggerMode;
    }
    PropertyI64 getTriggerSource( void ) const
    {
        return triggerSource;
    }
    PropertyI64 getTriggerActivation( void ) const
    {
        return triggerActivation;
    }
    PropertyI getTriggerDivider( void ) const
    {
        return triggerDivider;
    }
    PropertyF getTriggerDelayAbs_us( void ) const
    {
        return triggerDelayAbs_us;
    }
    PropertyI getTriggerDelayLines( void ) const
    {
        return triggerDelayLines;
    }
    PropertyI64DeviceTriggerOverlap getTriggerOverlap( void ) const
    {
        return triggerOverlap;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Properties to define the result images format.
/**
 *  This class provides properties allowing the user to define how the resulting
 *  image should be created. Things like the width and height of the captured image
 *  can be defined here for example.
 */
class ImageDestination : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::ImageDestination object.
    explicit ImageDestination(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : ComponentCollection( pDev ), pixelFormat(),
        scalerMode(), scalerInterpolationMode(), imageWidth(), imageHeight()
    {
        DeviceComponentLocator locator( pDev, dltSetting, settingName );
        locator.bindSearchBase( locator.searchbase_id(), "ImageDestination" );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( pixelFormat, "PixelFormat" );
        locator.bindComponent( scalerMode, "ScalerMode" );
        locator.bindComponent( scalerInterpolationMode, "ScalerInterpolationMode" );
        locator.bindComponent( imageWidth, "ImageWidth" );
        locator.bindComponent( imageHeight, "ImageHeight" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the pixel format of the resulting image.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TImageDestinationPixelFormat.
     */
    PropertyIImageDestinationPixelFormat pixelFormat;
    /// \brief An integer property defining whether the image is scaled or not.
    /**
     *  \note
     *  This feature is available for every device! However currently only \b mvDELTA / \b mvSIGMA
     *  devices provide hardware support for \b REDUCING the image size. For every other device this
     *  will be done by a software filter and therefore will introduce additional CPU load. Increasing
     *  the image size will always be done in software. When using the scaler the property
     *  \b mvIMPACT::acquire::StatisticsBase::imageProcTime_s can be used to observe the additional
     *  CPU time needed for the image processing.
     */
    PropertyIScalerMode scalerMode;
    /// \brief An interger property defining the interpolation method used when the scaler is active.
    /**
     *  \note
     *  Whenever the property \b mvIMPACT::acquire::ImageDestination::scalerMode is set to
     *  \b mvIMPACT::acquire::smOff modifying or reading this property will have no effect.
     *  Call \b Component::isVisible to find out whether this property is currently active or not.
     *
     *  \note
     *  Please see remarks under \b mvIMPACT::acquire::ImageDestination::scalerMode. The interpolation mode
     *  currently will only be used when scaling is \b NOT done by the hardware.
     */
    PropertyIScalerInterpolationMode scalerInterpolationMode;
    /// \brief An integer property defining the width of the scaled image.
    /**
     *  \note
     *  Whenever the property \b mvIMPACT::acquire::ImageDestination::scalerMode is set to
     *  \b mvIMPACT::acquire::smOff modifying or reading this property will have no effect.
     *  Call \b Component::isVisible to find out whether this property is currently active or not.
     *
     *  \note
     *  Please see remarks under \b mvIMPACT::acquire::ImageDestination::scalerMode.
     */
    PropertyI imageWidth;
    /// \brief An integer property defining the height of the scaled image.
    /**
     *  \note
     *  Whenever the property \b mvIMPACT::acquire::ImageDestination::scalerMode is set to
     *  \b mvIMPACT::acquire::smOff modifying or reading this property will have no effect.
     *  Call \b Component::isVisible to find out whether this property is currently active or not.
     *
     *  \note
     *  Please see remarks under \b mvIMPACT::acquire::ImageDestination::scalerMode.
     */
    PropertyI imageHeight;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIImageDestinationPixelFormat getPixelFormat( void ) const
    {
        return pixelFormat;
    }
    PropertyIScalerMode getScalerMode( void ) const
    {
        return scalerMode;
    }
    PropertyIScalerInterpolationMode getScalerInterpolationMode( void ) const
    {
        return scalerInterpolationMode;
    }
    PropertyI getImageWidth( void ) const
    {
        return imageWidth;
    }
    PropertyI getImageHeight( void ) const
    {
        return imageHeight;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Properties for adjusting the colors during a Bayer conversion.
/**
 *  This class provides properties to adjust the parameters needed for a
 *  Bayer conversion.
 *
 *  \note
 *  Objects of this class can't be constructed directly. Its parameters can
 *  be accessed via an instance of a class derived from \b mvIMPACT::acquire::ImageProcessing.
 */
class WhiteBalanceSettings : public ComponentCollection
//-----------------------------------------------------------------------------
{
    // only ImageProcessing objects can create these objects, as these objects are
    // fixed parts of that class.
    friend class ImageProcessing;
    explicit WhiteBalanceSettings( HOBJ hObj ): ComponentCollection( hObj ), WBAoiMode(), aoiHeight(), aoiStartX(), aoiStartY(),
        aoiWidth(), totalGain(), redGain(), greenGain(), blueGain(),  WBResult()
    {
        ComponentLocator locator( hObj );
        locator.bindComponent( WBAoiMode, "WBAoiMode" );
        locator.bindComponent( totalGain, "TotalGain" );
        locator.bindComponent( redGain, "RedGain" );
        locator.bindComponent( greenGain, "GreenGain" );
        locator.bindComponent( blueGain, "BlueGain" );
        locator.bindComponent( WBResult, "WBResult" );
        locator.bindSearchBase( locator.searchbase_id(), "WBAoi" );
        locator.bindComponent( aoiHeight, "H" );
        locator.bindComponent( aoiStartX, "X" );
        locator.bindComponent( aoiStartY, "Y" );
        locator.bindComponent( aoiWidth, "W" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the which area of the image is used
    /** for the calculation of the parameters.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAoiMode.
     */
    PropertyIAoiMode WBAoiMode;
    /// \brief An integer property defining the height of the AOI in pixel to be used for the calculation.
    /**
     *  \note
     *  This property will be visible only when \b mvIMPACT::acquire::WhiteBalanceSettings::WBAoiMode
     *  is set to \b mvIMPACT::acquire::amUseAoi.
     */
    PropertyI aoiHeight;
    /// \brief An integer property defining the X-offset of the AOI in pixel to be used for the calculation.
    /**
     *  \note
     *  This property will be visible only when \b mvIMPACT::acquire::WhiteBalanceSettings::WBAoiMode
     *  is set to \b mvIMPACT::acquire::amUseAoi.
     */
    PropertyI aoiStartX;
    /// \brief An integer property defining the Y-offset of the AOI in pixel to be used for the calculation.
    /**
     *  \note
     *  This property will be visible only when \b mvIMPACT::acquire::WhiteBalanceSettings::WBAoiMode
     *  is set to \b mvIMPACT::acquire::amUseAoi.
     */
    PropertyI aoiStartY;
    /// \brief An integer property defining the width of the AOI in pixel to be used for the calculation.
    /**
     *  \note
     *  This property will be visible only when \b mvIMPACT::acquire::WhiteBalanceSettings::WBAoiMode
     *  is set to \b mvIMPACT::acquire::amUseAoi.
     */
    PropertyI aoiWidth;
    /// \brief A float property defining the overall gain.
    PropertyF totalGain;
    /// \brief A float property defining the gain for the red channel.
    PropertyF redGain;
    /// \brief A float property defining the gain for the green channel.
    PropertyF greenGain;
    /// \brief A float property defining the gain for the blue channel.
    PropertyF blueGain;
    /// \brief An enumerated integer property \b (read-only) containing the result of the last white balance calibration.
    /**
     *  After the \b mvIMPACT::acquire::Device has been initialised, the value of this property will be
     *  \b mvIMPACT::acquire::bwbrUnknown.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBayerWhiteBalanceResult.
     */
    PropertyIBayerWhiteBalanceResult WBResult;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIAoiMode getWBAoiMode( void ) const
    {
        return WBAoiMode;
    }
    PropertyI getAoiHeight( void ) const
    {
        return aoiHeight;
    }
    PropertyI getAoiStartX( void ) const
    {
        return aoiStartX;
    }
    PropertyI getAoiStartY( void ) const
    {
        return aoiStartY;
    }
    PropertyI getAoiWidth( void ) const
    {
        return aoiWidth;
    }
    PropertyF getTotalGain( void ) const
    {
        return totalGain;
    }
    PropertyF getRedGain( void ) const
    {
        return redGain;
    }
    PropertyF getGreenGain( void ) const
    {
        return greenGain;
    }
    PropertyF getBlueGain( void ) const
    {
        return blueGain;
    }
    PropertyIBayerWhiteBalanceResult getWBresult( void ) const
    {
        return WBResult;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Properties for configuring settings belonging to a certain channel of the GainOffsetKnee filter.
/**
 *  First the master offset will be added to all channels of the image, then each
 *  channels individual gain factor will be applied to the pixel of each channel and
 *  finally the channel specific offset will be added.
 *
 *  \image html GainOffsetKnee.png
 *
 *  \note
 *  Objects of this class can't be constructed directly. Its parameters can
 *  be accessed via an instance of a class derived from \b mvIMPACT::acquire::ImageProcessing.
 */
class GainOffsetKneeChannelParameters : public ComponentCollection
//-----------------------------------------------------------------------------
{
    // only ImageProcessing objects can create these objects, as these objects are
    // fixed parts of that class.
    friend class ImageProcessing;
    explicit GainOffsetKneeChannelParameters( HOBJ hObj ): ComponentCollection( hObj ), gain(), offset_pc()
    {
        ComponentLocator locator( hObj );
        locator.bindComponent( gain, "Gain" );
        locator.bindComponent( offset_pc, "Offset_pc" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief A float property that contains the channel specific gain to be applied to the selected channel of the image.
    /**
     *  This gain will be applied after the master offset but before the channel specific offset.
     */
    PropertyF gain;
    /// \brief A float property that contains the channel specific offset (in percent) to be applied to the selected channel of the image.
    /**
     *  This offset will be applied after the channel specific gain.
     */
    PropertyF offset_pc;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyF getGain( void ) const
    {
        return gain;
    }
    PropertyF getOffset_pc( void ) const
    {
        return offset_pc;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Properties for configuring settings belonging to a certain LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) to be applied to a captured image.
/**
 *  \note
 *  Objects of this class can't be constructed directly. Its parameters can
 *  be accessed via an instance of a class derived from \b mvIMPACT::acquire::ImageProcessing.
 */
class LUTParameters : public ComponentCollection
//-----------------------------------------------------------------------------
{
    // only ImageProcessing objects can create these objects, as these objects are
    // fixed parts of that class.
    friend class ImageProcessing;
    explicit LUTParameters( HOBJ hObj ): ComponentCollection( hObj ), gamma(),
        valueCount(), inputValues(), outputValues(), directValues()
    {
        ComponentLocator locator( hObj );
        locator.bindComponent( gamma, "Gamma" );
        locator.bindComponent( gammaAlpha, "GammaAlpha" );
        locator.bindComponent( gammaMode, "GammaMode" );
        locator.bindComponent( gammaStartThreshold, "GammaStartThreshold" );
        locator.bindComponent( valueCount, "ValueCount" );
        locator.bindComponent( inputValues, "InputValues" );
        locator.bindComponent( outputValues, "OutputValues" );
        locator.bindComponent( directValues, "DirectValues" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief A float property which can be used to set the gamma value.
    /**
     *  Gamma correction is explained e.g. in this Wikipedia articla: http://en.wikipedia.org/wiki/Gamma_correction.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmGamma.
     *
     *  The gamma value will be used to calculate the corresponding input -> output transformation LUT.
     *
     *  The following formula will be used:
     * \code
     *   ( ( 1 + gammaAlpha ) * inputValue^(1/gamma) ) - gammaAlpha
     * \endcode
     */
    PropertyF gamma;
    /// \brief A float property which can be used to set the gammaAlpha value.
    /**
     *  Refer to \b mvIMPACT::acquire::LUTParameters::gamma to see how this parameter affects the resulting
     *  LUT data.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmGamma.
     */
    PropertyF gammaAlpha;
    /// \brief An enumerated integer property which can be used to configure the LUT(<b>L</b>ook <b>U</b>p <b>T</b>able) gamma mode.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTGammaMode.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmGamma.
     */
    PropertyILUTGammaMode gammaMode;
    /// \brief An integer property which can be used to define a start threshold above which the gamma correction formula shall be used in \b mvIMPACT::acquire::LUTParameters::gammaMode \b mvIMPACT::acquire::LUTgmLinearStart.
    /**
     *  Values below that threshold will be calculated using linear interpolation.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmGamma or if \b mvIMPACT::acquire::LUTParameters::gammaMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTgmLinearStart.
     */
    PropertyI gammaStartThreshold;
    /// \brief An integer property to define the number of sampling points for interpolated LUT transformations.
    /**
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmInterpolated.
     *
     *  At least 2 sampling points are needed, the max. number of sampling points is defined by the bit depth of the input
     *  image, thus a 8 bit input image can have no more than 2^8 (256) sampling points. if the max. number of sampling points
     *  is used, the behaviour is the same a when working with \b mvIMPACT::acquire::ImageProcessing::LUTMode set to
     *  \b mvIMPACT::acquire::LUTmDirect.
     *
     *  \b EXAMPLE
     *
     *  Assuming 4 sampling points in \b mvIMPACT::acquire::LUTimThreshold mode for an 8 to 8 bit transformation where the following
     *  values are used:
     *
     *  <table>
     *  <tr><td class="header">index</td><td class="header">input value</td><td class="header">output value</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">0</td><td class="indexvalue">64</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">128</td><td class="indexvalue">255</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">192</td><td class="indexvalue">0</td></tr>
     *  <tr><td class="indexvalue">3</td><td class="indexvalue">255</td><td class="indexvalue">0</td></tr>
     *  </table>
     *
     *  This will result in the following transfer function characteristic:
     *
     *  \image html LUT_ThresholdInterpolation_4SamplingPoints.png
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  \b The following C++ code would be needed to achieve this behaviour for mono video sources:
     *
     * \code
     *  //-----------------------------------------------------------------------------
     *  void fn( ImageProcessing& ip, TLUTImplemention LUTImplementation )
     *  //-----------------------------------------------------------------------------
     *  {
     *    ip.LUTEnable.write( bTrue );
     *    ip.LUTMode.write( LUTmInterpolated );
     *    ip.LUTImplementation.write( LUTImplementation );
     *    ip.LUTInterpolationMode.write( LUTimThreshold );
     *    switch( LUTImplementation )
     *    {
     *      case LUTiHardware:
     *        ipLUTMappingHardware.write( LUTm8To8 );
     *        break;
     *      case LUTiSoftware:
     *        ipLUTMappingSoftware.write( LUTm8To8 );
     *        break;
     *     }
     *     LUTParameters& LUTAccess = ip.getLUTParameter( 0 );
     *     LUTAccess.valueCount.write( 4 );
     *     vector<int> v(4);
     *     v[0] = 0;
     *     v[1] = 128;
     *     v[2] = 192;
     *     v[3] = 255;
     *     LUTAccess.inputValues.write( v );
     *     v[0] = 64;
     *     v[1] = 255;
     *     v[2] = 0;
     *     v[3] = 0;
     *     LUTAccess.outputValues.write( v );
     *  }
     * \endcode
     *  \endif
     *  Assuming 3 sampling points in \b mvIMPACT::acquire::LUTimLinear mode for an 8 to 8 bit transformation where the following
     *  values are used:
     *
     *  <table>
     *  <tr><td class="header">index</td><td class="header">input value</td><td class="header">output value</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">0</td><td class="indexvalue">64</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">128</td><td class="indexvalue">255</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">255</td><td class="indexvalue">0</td></tr>
     *  </table>
     *
     *  This will result in the following transfer function characteristic:
     *
     *  \image html LUT_linearInterpolation_3SamplingPoints.png
     *
     *  Assuming 4 sampling points in \b mvIMPACT::acquire::LUTimCubic mode for an 8 to 8 bit transformation where the following
     *  values are used:
     *
     *  <table>
     *  <tr><td class="header">index</td><td class="header">input value</td><td class="header">output value</td></tr>
     *  <tr><td class="indexvalue">0</td><td class="indexvalue">0</td><td class="indexvalue">0</td></tr>
     *  <tr><td class="indexvalue">1</td><td class="indexvalue">128</td><td class="indexvalue">255</td></tr>
     *  <tr><td class="indexvalue">2</td><td class="indexvalue">192</td><td class="indexvalue">192</td></tr>
     *  <tr><td class="indexvalue">3</td><td class="indexvalue">255</td><td class="indexvalue">0</td></tr>
     *  </table>
     *
     *  This will result in the following transfer function characteristic:
     *
     *  \image html LUT_CubicInterpolation_4SamplingPoints.png
     */
    PropertyI valueCount;
    /// \brief An integer property to define the input values for sampling points for interpolated LUT transformations.
    /**
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmInterpolated.
     *
     *  The number of values stored by this property can be changed by writing to the property \b mvIMPACT::acquire::LUTParameters::valueCount.
     *  The documentation of this property also provides some example values and resulting transfer characteristics.
     */
    PropertyI inputValues;
    /// \brief An integer property to define the output values for sampling points for interpolated LUT transformations.
    /**
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmInterpolated.
     *
     *  The number of values stored by this property can be changed by writing to the property \b mvIMPACT::acquire::LUTParameters::valueCount.
     *  The documentation of this property also provides some example values and resulting transfer characteristics.
     */
    PropertyI outputValues;
    /// \brief An integer property which can be used to directly define a LUT.
    /**
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmDirect.
     *
     *  This property will provide a complete LUT thus e.g. in a LUT mapping to a 10 bit output this property will store 2^10 (1024)
     *  values that can be accessed and modified by the user.
     */
    PropertyI directValues;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyF getGamma( void ) const
    {
        return gamma;
    }
    PropertyF getGammaAlpha( void ) const
    {
        return gammaAlpha;
    }
    PropertyILUTGammaMode getGammaMode( void ) const
    {
        return gammaMode;
    }
    PropertyI getGammaStartThreshold( void ) const
    {
        return gammaStartThreshold;
    }
    PropertyI getValueCount( void ) const
    {
        return valueCount;
    }
    PropertyI getInputValues( void ) const
    {
        return inputValues;
    }
    PropertyI getOutputValues( void ) const
    {
        return outputValues;
    }
    PropertyI getDirectValues( void ) const
    {
        return directValues;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Properties for configuring settings belonging to the mirror filter that processes a certain channel of a captured image.
/**
 *  Properties in this class will only have influence on the capture process if
 *  \b mvIMPACT::acquire::ImageProcessing::mirrorOperationMode is set to \b mvIMPACT::acquire::momChannelBased.
 *
 *  \note
 *  Objects of this class can't be constructed directly. Its parameters can
 *  be accessed via an instance of a class derived from \b mvIMPACT::acquire::ImageProcessing.
 */
class MirrorParameters : public ComponentCollection
//-----------------------------------------------------------------------------
{
    // only ImageProcessing objects can create these objects, as these objects are
    // fixed parts of that class.
    friend class ImageProcessing;
    explicit MirrorParameters( HOBJ hObj ): ComponentCollection( hObj ), mirrorMode()
    {
        ComponentLocator locator( hObj );
        locator.bindComponent( mirrorMode, "MirrorMode" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the mirror mode to be applied to this channel of the image.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TMirrorMode.
     */
    PropertyIMirrorMode mirrorMode;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIMirrorMode getMirrorMode( void ) const
    {
        return mirrorMode;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Base class for image processing related properties.
/**
 *  This class provides access to properties, which define how the image shall
 *  be processed after it has been captured but before it will be handed back to
 *  the user.
 */
class ImageProcessing : public ComponentCollection
//-----------------------------------------------------------------------------
{
    typedef std::vector<GainOffsetKneeChannelParameters*> GainOffsetKneeParameterContainer;
    typedef std::vector<LUTParameters*> LUTParameterContainer;
    typedef std::vector<MirrorParameters*> MirrorParameterContainer;
    typedef std::vector<WhiteBalanceSettings*> WBSettingsContainer;
#   ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        HDRV                             m_hDrv;
        HLIST                            m_hSetting;
        GainOffsetKneeParameterContainer m_GainOffsetKneeParameters;
        LUTParameterContainer            m_LUTParameters;
        MirrorParameterContainer         m_MirrorParameters;
        WBSettingsContainer              m_WBUserSettings;
        unsigned int                     m_refCnt;
        ReferenceCountedData( HDRV hDrv, HLIST hSetting ) : m_hDrv( hDrv ), m_hSetting( hSetting ),
            m_GainOffsetKneeParameters(), m_LUTParameters(), m_MirrorParameters(), m_WBUserSettings(), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            const GainOffsetKneeParameterContainer::size_type GainOffsetKneeParameterCnt = m_GainOffsetKneeParameters.size();
            for( GainOffsetKneeParameterContainer::size_type h = 0; h < GainOffsetKneeParameterCnt; h++ )
            {
                delete m_GainOffsetKneeParameters[h];
            }
            const LUTParameterContainer::size_type LUTParametersCnt = m_LUTParameters.size();
            for( LUTParameterContainer::size_type i = 0; i < LUTParametersCnt; i++ )
            {
                delete m_LUTParameters[i];
            }
            const MirrorParameterContainer::size_type MirrorParametersCnt = m_MirrorParameters.size();
            for( MirrorParameterContainer::size_type j = 0; j < MirrorParametersCnt; j++ )
            {
                delete m_MirrorParameters[j];
            }
            const WBSettingsContainer::size_type WBSettingsCnt = m_WBUserSettings.size();
            for( WBSettingsContainer::size_type k = 0; k < WBSettingsCnt; k++ )
            {
                delete m_WBUserSettings[k];
            }
        }
    }* m_pRefData;
#   endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    void bindPublicProperties( void )
    //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( colorProcessing, "ColorProcessing" );
        locator.bindComponent( bayerConversionMode, "BayerConversionMode" );
        locator.bindComponent( whiteBalance, "WhiteBalance" );
        locator.bindComponent( whiteBalanceCalibration, "WhiteBalanceCalibration" );
        locator.bindComponent( filter, "Filter" );
        if( locator.findComponent( "Mirror" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "Mirror" );
            locator.bindComponent( mirrorOperationMode, "MirrorOperationMode" );
            locator.bindComponent( mirrorModeGlobal, "MirrorModeGlobal" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "DefectivePixelsFilter" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "DefectivePixelsFilter" );
            locator.bindComponent( defectivePixelsFilterMode, "Mode" );
            locator.bindComponent( defectivePixelsFilterLeakyPixelDeviation_ADCLimit, "LeakyPixelDeviation_ADCLimit" );
            locator.bindComponent( defectivePixelsFilterColdPixelDeviation_pc, "ColdPixelDeviation_pc" );
            locator.bindComponent( defectivePixelsFound, "DefectivePixelsFound" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "FlatFieldFilter" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "FlatFieldFilter" );
            locator.bindComponent( flatFieldFilterMode, "Mode" );
            locator.bindComponent( flatFieldFilterCalibrationImageCount, "CalibrationImageCount" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "DarkCurrentFilter" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "DarkCurrentFilter" );
            locator.bindComponent( darkCurrentFilterMode, "Mode" );
            locator.bindComponent( darkCurrentFilterCalibrationImageCount, "CalibrationImageCount" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "GainOffsetKnee" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "GainOffsetKnee" );
            locator.bindComponent( gainOffsetKneeEnable, "GainOffsetKneeEnable" );
            locator.bindComponent( gainOffsetKneeMasterOffset_pc, "GainOffsetKneeMasterOffset_pc" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "LUTOperations" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "LUTOperations" );
            locator.bindComponent( LUTEnable, "LUTEnable" );
            locator.bindComponent( LUTMode, "LUTMode" );
            locator.bindComponent( LUTInterpolationMode, "LUTInterpolationMode" );
            locator.bindComponent( LUTImplementation, "LUTImplementation" );
            locator.bindComponent( LUTMappingHardware, "LUTMappingHardware" );
            locator.bindComponent( LUTMappingSoftware, "LUTMappingSoftware" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "TapSort" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "TapSort" );
            locator.bindComponent( tapSortEnable, "TapSortEnable" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "ChannelSplit" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "ChannelSplit" );
            locator.bindComponent( channelSplitEnable, "ChannelSplitEnable" );
            locator.bindComponent( channelSplitMode, "ChannelSplitMode" );
            locator.bindComponent( channelSplitChannelIndex, "ChannelSplitChannelIndex" );
            locator.bindComponent( channelSplitDeinterlaceEnable, "ChannelSplitDeinterlaceEnable" );
        }
        locator.bindSearchBase( m_hRoot );
        if( locator.findComponent( "ColorTwist" ) != INVALID_ID )
        {
            locator.bindSearchBase( m_hRoot, "ColorTwist" );
            locator.bindComponent( colorTwistInputCorrectionMatrixEnable, "ColorTwistInputCorrectionMatrixEnable" );
            locator.bindComponent( colorTwistInputCorrectionMatrixMode, "ColorTwistInputCorrectionMatrixMode" );
            locator.bindComponent( colorTwistInputCorrectionMatrixRow0, "ColorTwistInputCorrectionMatrixRow0" );
            locator.bindComponent( colorTwistInputCorrectionMatrixRow1, "ColorTwistInputCorrectionMatrixRow1" );
            locator.bindComponent( colorTwistInputCorrectionMatrixRow2, "ColorTwistInputCorrectionMatrixRow2" );
            locator.bindComponent( colorTwistEnable, "ColorTwistEnable" );
            locator.bindComponent( colorTwistRow0, "ColorTwistRow0" );
            locator.bindComponent( colorTwistRow1, "ColorTwistRow1" );
            locator.bindComponent( colorTwistRow2, "ColorTwistRow2" );
            locator.bindComponent( colorTwistOutputCorrectionMatrixEnable, "ColorTwistOutputCorrectionMatrixEnable" );
            locator.bindComponent( colorTwistOutputCorrectionMatrixMode, "ColorTwistOutputCorrectionMatrixMode" );
            locator.bindComponent( colorTwistOutputCorrectionMatrixRow0, "ColorTwistOutputCorrectionMatrixRow0" );
            locator.bindComponent( colorTwistOutputCorrectionMatrixRow1, "ColorTwistOutputCorrectionMatrixRow1" );
            locator.bindComponent( colorTwistOutputCorrectionMatrixRow2, "ColorTwistOutputCorrectionMatrixRow2" );
            locator.bindComponent( colorTwistResultingMatrixRow0, "ColorTwistResultingMatrixRow0" );
            locator.bindComponent( colorTwistResultingMatrixRow1, "ColorTwistResultingMatrixRow1" );
            locator.bindComponent( colorTwistResultingMatrixRow2, "ColorTwistResultingMatrixRow2" );
        }
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
public:
    /// brief Constructs a new \b mvIMPACT::acquire::ImageProcessing object.
    explicit ImageProcessing(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : ComponentCollection( pDev ), m_pRefData( 0 ),
        colorProcessing(), bayerConversionMode(), whiteBalance(), whiteBalanceCalibration(), filter(),
        mirrorOperationMode(), mirrorModeGlobal(),
        defectivePixelsFilterMode(), defectivePixelsFilterLeakyPixelDeviation_ADCLimit(),
        defectivePixelsFilterColdPixelDeviation_pc(), defectivePixelsFound(),
        flatFieldFilterMode(), flatFieldFilterCalibrationImageCount(), darkCurrentFilterMode(),
        darkCurrentFilterCalibrationImageCount(),
        gainOffsetKneeEnable(), gainOffsetKneeMasterOffset_pc(),
        LUTEnable(), LUTMode(), LUTInterpolationMode(),
        LUTImplementation(), LUTMappingHardware(), LUTMappingSoftware(),
        tapSortEnable(), channelSplitEnable(), channelSplitMode(), channelSplitChannelIndex(), channelSplitDeinterlaceEnable(),
        colorTwistInputCorrectionMatrixEnable(), colorTwistInputCorrectionMatrixMode(),
        colorTwistInputCorrectionMatrixRow0(), colorTwistInputCorrectionMatrixRow1(), colorTwistInputCorrectionMatrixRow2(),
        colorTwistEnable(), colorTwistRow0(), colorTwistRow1(), colorTwistRow2(),
        colorTwistOutputCorrectionMatrixEnable(), colorTwistOutputCorrectionMatrixMode(),
        colorTwistOutputCorrectionMatrixRow0(), colorTwistOutputCorrectionMatrixRow1(), colorTwistOutputCorrectionMatrixRow2(),
        colorTwistResultingMatrixRow0(), colorTwistResultingMatrixRow1(), colorTwistResultingMatrixRow2()
    {
        DeviceComponentLocator locator( pDev, dltSetting, settingName );
        m_pRefData = new ReferenceCountedData( pDev->hDrv(), locator.searchbase_id() );
        locator.bindSearchBase( m_pRefData->m_hSetting, "ImageProcessing" );
        m_hRoot = locator.searchbase_id();
        bindPublicProperties();
        int number = 1;
        HOBJ hList = INVALID_ID;
        // create the WB user parameter objects for this setting
        do
        {
            std::ostringstream oss;
            oss << "WhiteBalanceSettings-" << number++;
            if( ( hList = locator.findComponent( oss.str() ) ) != INVALID_ID )
            {
                m_pRefData->m_WBUserSettings.push_back( new WhiteBalanceSettings( hList ) );
            }
        }
        while( hList != INVALID_ID );

        // create the GainOffsetKnee parameter objects for this setting
        if( locator.findComponent( "GainOffsetKnee" ) != INVALID_ID )
        {
            // this driver offers GainOffsetKnee support
            locator.bindSearchBase( locator.searchbase_id(), "GainOffsetKnee" );
            locator.bindSearchBase( locator.searchbase_id(), "GainOffsetKneeChannels" );
            number = 0;
            do
            {
                std::ostringstream oss;
                oss << "Channel-" << number++;
                if( ( hList = locator.findComponent( oss.str() ) ) != INVALID_ID )
                {
                    m_pRefData->m_GainOffsetKneeParameters.push_back( new GainOffsetKneeChannelParameters( hList ) );
                }
            }
            while( hList != INVALID_ID );
        }

        locator.bindSearchBase( m_hRoot );
        // create the Mirror parameter objects for this setting
        if( locator.findComponent( "Mirror" ) != INVALID_ID )
        {
            // this driver offers Mirror support
            locator.bindSearchBase( locator.searchbase_id(), "Mirror" );
            locator.bindSearchBase( locator.searchbase_id(), "MirrorChannels" );
            number = 0;
            do
            {
                std::ostringstream oss;
                oss << "Channel-" << number++;
                if( ( hList = locator.findComponent( oss.str() ) ) != INVALID_ID )
                {
                    m_pRefData->m_MirrorParameters.push_back( new MirrorParameters( hList ) );
                }
            }
            while( hList != INVALID_ID );
        }

        locator.bindSearchBase( m_hRoot );
        // create the LUT parameter objects for this setting
        if( locator.findComponent( "LUTOperations" ) != INVALID_ID )
        {
            // this driver offers LUT support
            locator.bindSearchBase( locator.searchbase_id(), "LUTOperations" );
            locator.bindSearchBase( locator.searchbase_id(), "LUTs" );
            number = 0;
            do
            {
                std::ostringstream oss;
                oss << "LUT-" << number++;
                if( ( hList = locator.findComponent( oss.str() ) ) != INVALID_ID )
                {
                    m_pRefData->m_LUTParameters.push_back( new LUTParameters( hList ) );
                }
            }
            while( hList != INVALID_ID );
        }
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageProcessing from and exisiting one.
    explicit ImageProcessing(
        /// [in] A constant reference to the \b mvIMPACT::acquire::ImageProcessing object, this object shall be created from
        const ImageProcessing& src ) : ComponentCollection( src ), m_pRefData( src.m_pRefData ),
        colorProcessing( src.colorProcessing ), bayerConversionMode( src.bayerConversionMode ), whiteBalance( src.whiteBalance ),
        whiteBalanceCalibration( src.whiteBalanceCalibration ), filter( src.filter ),
        mirrorOperationMode( src.mirrorOperationMode ),
        mirrorModeGlobal( src.mirrorModeGlobal ), defectivePixelsFilterMode( src.defectivePixelsFilterMode ),
        defectivePixelsFilterLeakyPixelDeviation_ADCLimit( src.defectivePixelsFilterLeakyPixelDeviation_ADCLimit ),
        defectivePixelsFilterColdPixelDeviation_pc( src.defectivePixelsFilterColdPixelDeviation_pc ),
        defectivePixelsFound( src.defectivePixelsFound ),
        flatFieldFilterMode( src.flatFieldFilterMode ),
        flatFieldFilterCalibrationImageCount( src.flatFieldFilterCalibrationImageCount ),
        darkCurrentFilterMode( src.darkCurrentFilterMode ),
        darkCurrentFilterCalibrationImageCount( src.darkCurrentFilterCalibrationImageCount ),
        gainOffsetKneeEnable( src.gainOffsetKneeEnable ), gainOffsetKneeMasterOffset_pc( src.gainOffsetKneeMasterOffset_pc ),
        LUTEnable( src.LUTEnable ), LUTMode( src.LUTMode ), LUTInterpolationMode( src.LUTInterpolationMode ),
        LUTImplementation( src.LUTImplementation ), LUTMappingHardware( src.LUTMappingHardware ), LUTMappingSoftware( src.LUTMappingSoftware ),
        tapSortEnable( src.tapSortEnable ), channelSplitEnable( src.channelSplitEnable ), channelSplitMode( src.channelSplitMode ),
        channelSplitChannelIndex( src.channelSplitChannelIndex ), channelSplitDeinterlaceEnable( src.channelSplitDeinterlaceEnable ),
        colorTwistInputCorrectionMatrixEnable( src.colorTwistInputCorrectionMatrixEnable ),
        colorTwistInputCorrectionMatrixMode( src.colorTwistInputCorrectionMatrixMode ),
        colorTwistInputCorrectionMatrixRow0( src.colorTwistInputCorrectionMatrixRow0 ),
        colorTwistInputCorrectionMatrixRow1( src.colorTwistInputCorrectionMatrixRow1 ),
        colorTwistInputCorrectionMatrixRow2( src.colorTwistInputCorrectionMatrixRow2 ),
        colorTwistEnable( src.colorTwistEnable ),
        colorTwistRow0( src.colorTwistRow0 ), colorTwistRow1( src.colorTwistRow1 ), colorTwistRow2( src.colorTwistRow2 ),
        colorTwistOutputCorrectionMatrixEnable( src.colorTwistOutputCorrectionMatrixEnable ),
        colorTwistOutputCorrectionMatrixMode( src.colorTwistOutputCorrectionMatrixMode ),
        colorTwistOutputCorrectionMatrixRow0( src.colorTwistOutputCorrectionMatrixRow0 ),
        colorTwistOutputCorrectionMatrixRow1( src.colorTwistOutputCorrectionMatrixRow1 ),
        colorTwistOutputCorrectionMatrixRow2( src.colorTwistOutputCorrectionMatrixRow2 ),
        colorTwistResultingMatrixRow0( src.colorTwistResultingMatrixRow0 ),
        colorTwistResultingMatrixRow1( src.colorTwistResultingMatrixRow1 ),
        colorTwistResultingMatrixRow2( src.colorTwistResultingMatrixRow2 )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor.
    virtual ~ImageProcessing()
    {
        dealloc();
    }
#ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::ImageProcessing objects
    ImageProcessing& operator=( const ImageProcessing& rhs )
    {
        if( this != &rhs )
        {
            ComponentCollection::operator=( rhs );
            dealloc();
            m_pRefData = rhs.m_pRefData;
            ++m_pRefData->m_refCnt;
            bindPublicProperties();
        }
        return *this;
    }
#endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining what kind of color processing shall be applied to the raw image data.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TColorProcessingMode.
     */
    PropertyIColorProcessingMode colorProcessing;
    /// \brief An enumerated integer property defining what kind of algorithm shall be used during Bayer to RGB conversion.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBayerConversionMode.
     */
    PropertyIBayerConversionMode bayerConversionMode;
    /// \brief An integer property defining the parameter set to be used to perform the white balance correction.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TWhiteBalanceParameter.
     */
    PropertyIWhiteBalanceParameter whiteBalance;
    /// \brief An enumarated integer property defining the mode used for white balance calibration.
    /**
     *  This property can be used to define the way a white balance calibration shall be performed.
     *  Currently only the \b mvIMPACT::acquire::wbcmNextFrame mode is supported,
     *  but new modes might appear in later revisions of the interface.
     *
     *  When this property is set to \b mvIMPACT::acquire::wbcmNextFrame the next
     *  image captured will be used as a reference to calculate the gain factors for the
     *  \e red, \e green and \e blue channel. So make sure when the next image is captured the camera
     *  looks on a plain white or slightly grey surface in order to obtain correct results. After
     *  the calibration has been done, the property will be reset to \b mvIMPACT::acquire::wbcmOff
     *  again.
     *
     *  \note
     *  Performing this kind of white balance calibration will only affect Bayer color sensors and will
     *  only have any visual effect on the image if one of the user definable parameter sets has been
     *  selected before. This can be done by modifying the property \b mvIMPACT::acquire::ImageProcessing::whiteBalance.
     *  Afterwards the calculated factors will be stored in the \b mvIMPACT::acquire::WhiteBalanceSettings selected.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TWhiteBalanceCalibrationMode.
     */
    PropertyIWhiteBalanceCalibrationMode whiteBalanceCalibration;
    /// \brief An enumerated integer property defining the filter to be applied to the image before it is transferred to the user.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TImageProcessingFilter.
     */
    PropertyIImageProcessingFilter filter;
    /// \brief An enumerated integer property defining the general mode the mirror filter will work in.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TMirrorOperationMode.
     */
    PropertyIMirrorOperationMode mirrorOperationMode;
    /// \brief An enumerated integer property defining the mirror mode to be applied to ALL channels of the image.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TMirrorMode.
     *
     *  \note This property will only be visible, when \b mvIMPACT::acquire::ImageProcessing::mirrorOperationMode is set
     *  to \b mvIMPACT::acquire::momGlobal.
     */
    PropertyIMirrorMode mirrorModeGlobal;
    /// \brief An enumerated integer property defining the operation mode of the defective pixels filter.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDefectivePixelsFilterMode.
     */
    PropertyIDefectivePixelsFilterMode defectivePixelsFilterMode;
    /// \brief An integer property defining the max. allowed offset from the average grey value for a pixel during calibration.
    /**
     *  When this filter is calibrated each pixel producing a grey value higher than the average grey value plus
     *  the value of this property will be considered as a leaky pixel and therefore will be replaced by
     *  a value depending on the operation mode of the filter.
     */
    PropertyI defectivePixelsFilterLeakyPixelDeviation_ADCLimit;
    /// \brief An integer property defining the max. allowed deviation for a pixel in per cent from the average grey value.
    /**
     *  If this filter is active every pixel that during the calibration produces a grey value which is
     *  lower than the average grey value of the image minus the allowed deviation in per cent will be
     *  considered as a cold pixel and therefore will be replaced by a value depending on the operation
     *  mode of the filter.
     */
    PropertyI defectivePixelsFilterColdPixelDeviation_pc;
    /// \brief An integer property \b (read-only) containing the number of pixels considered as being defective with respect to the
    /// last calibration run.
    /**
     *  \note
     *  In order to collect \b ALL defective pixels the list of detected pixels in not emptied each time a new calibration
     *  is started. In order to get rid of currently detected pixels an application must set the property
     *  \b mvIMPACT::acquire::ImageProcessing::defectivePixelsFilterMode to \b mvIMPACT::acquire::dpfmResetCalibration
     *  and then capture a fresh image. Only then all currently detected pixels will be discarded.
     */
    PropertyI defectivePixelsFound;
    /// \brief An enumerated integer property defining the operation mode of the flat field correction filter.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TFlatFieldFilterMode.
     */
    PropertyIFlatFieldFilterMode flatFieldFilterMode;
    /// \brief An integer property defining the number of consecutive images to take into account during the calibration of the flat field filter.
    PropertyI flatFieldFilterCalibrationImageCount;
    /// \brief An enumerated integer property defining the operation mode of the dark current filter.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDarkCurrentFilterMode.
     */
    PropertyIDarkCurrentFilterMode darkCurrentFilterMode;
    /// \brief An integer property defining the number of consecutive images to take into account during calibration of the dark current filter.
    PropertyI darkCurrentFilterCalibrationImageCount;
    /// \brief An enumerated integer property which can be used to enable the Gain, Offset, Knee filter.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean gainOffsetKneeEnable;
    /// \brief A float property that contains master offset(in percent).
    /**
     *  This offset will be applied before the channel specific gain.
     */
    PropertyF gainOffsetKneeMasterOffset_pc;
    /// \brief An enumerated integer property which can be used to enable LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) transformations.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean LUTEnable;
    /// \brief An enumerated integer property which can be used to configure the LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) transformation mode.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTMode.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse.
     */
    PropertyILUTMode LUTMode;
    /// \brief An enumerated integer property which can be used to configure the LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) interpolation mode.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTInterpolationMode.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTMode is \b NOT set to
     *  \b mvIMPACT::acquire::LUTmInterpolated.
     */
    PropertyILUTInterpolationMode LUTInterpolationMode;
    /// \brief An enumerated integer property which can be used to configure the LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) implementation.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTImplementation.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse.
     */
    PropertyILUTImplementation LUTImplementation;
    /// \brief An enumerated integer property which can be used to configure the LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) mapping mode for LUT transformations done by the hardware.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTMapping.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTImplementation is \b NOT set to
     *  \b mvIMPACT::acquire::LUTiHardware.
     */
    PropertyILUTMapping LUTMappingHardware;
    /// \brief An enumerated integer property which can be used to configure the LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) mapping mode for LUT transformations done optimized software algorithms.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TLUTMapping.
     *
     *  \note This property will be invisible when \b mvIMPACT::acquire::ImageProcessing::LUTEnable is set to
     *  \b mvIMPACT::acquire::bFalse or if \b mvIMPACT::acquire::ImageProcessing::LUTImplementation is \b NOT set to
     *  \b mvIMPACT::acquire::LUTiSoftware.
     */
    PropertyILUTMapping LUTMappingSoftware;
    /// \brief An enumerated integer property which can be used to enable/disable automatic data re-ordering from image sources delivering data from multiple taps that require re-ordering to reconstruct the image.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean tapSortEnable;
    /// \brief An enumerated integer property which can be used to enable/disable automatic data re-ordering or extraction from multi-channel images.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean channelSplitEnable;
    /// \brief An enumerated integer property which can be used to define how data from multi-channel images shall be re-ordered.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TChannelSplitMode.
     */
    PropertyIChannelSplitMode channelSplitMode;
    /// \brief An integer property which can be used to define which channel of the image shall be extracted.
    /**
     *  This property will only be visible if \b mvIMPACT::acquire::ImageProcessing::channelSplitMode is set
     *  to \b mvIMPACT::acquire::csmExtractSingle.
     */
    PropertyI channelSplitChannelIndex;
    /// \brief An enumerated integer property which can be used to enable/disable data re-ordering for interlaced images.
    /**
     *  Enabling this feature will cause interlaced images in a way that the 2 fields will be display next to each other. This
     *  will modify the original image in an image with twice the width but half the height.
     *
     *  \note
     *  Right now this property will only result in correct results for images, that are line wise interlaced, thus
     *  some special sensors that e.g. transfer Bayer interlaced data (2 lines field 0, 2 lines field even, etc.)
     *  can not be re-arranged in a meaningful way.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean channelSplitDeinterlaceEnable;
    /// \brief An enumerated integer property which can be used to enable/disable the color twist input correction matrix.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean colorTwistInputCorrectionMatrixEnable;
    /// \brief An enumerated integer property which can be used to select an input color correction matrix.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TColorTwistInputCorrectionMatrixMode.
     */
    PropertyIColorTwistInputCorrectionMatrixMode colorTwistInputCorrectionMatrixMode;
    /// \brief The first row of the input color correction matrix.
    /**
     *  This property will store the first row of a user defined 3x3 input color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::cticmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistInputCorrectionMatrixRow0;
    /// \brief The second row of the input color correction matrix.
    /**
     *  This property will store the second row of a user defined 3x3 input color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::cticmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistInputCorrectionMatrixRow1;
    /// \brief The third row of the input color correction matrix.
    /**
     *  This property will store the third row of a user defined 3x3 input color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::cticmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistInputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistInputCorrectionMatrixRow2;
    /// \brief An enumerated integer property which can be used to enable/disable the color twist matrix.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  The color twist filter can be used to apply a linear transformation to a 3 channel image. Each pixel will first
     *  be multiplied by a 3x3 matrix and can then be added to an offset triplet.
     */
    PropertyIBoolean colorTwistEnable;
    /// \brief The first row of the color twist matrix.
    /**
     *  This property will store 4 values. The first 3 components for the first row of the 3x4 matrix, the last component is the
     *  offset of this row.
     */
    PropertyF colorTwistRow0;
    /// \brief The second row of the color twist matrix.
    /**
     *  This property will store 4 values. The first 3 components for the second row of the 3x4 matrix, the last component is the
     *  offset of this row.
     */
    PropertyF colorTwistRow1;
    /// \brief The third row of the color twist matrix.
    /**
     *  This property will store 4 values. The first 3 components for the third row of the 3x4 matrix, the last component is the
     *  offset of this row.
     */
    PropertyF colorTwistRow2;
    /// \brief An enumerated integer property which can be used to enable/disable the color twist output correction matrix.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean colorTwistOutputCorrectionMatrixEnable;
    /// \brief An enumerated integer property which can be used to select an output color correction matrix.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TColorTwistOutputCorrectionMatrixMode.
     */
    PropertyIColorTwistOutputCorrectionMatrixMode colorTwistOutputCorrectionMatrixMode;
    /// \brief The first row of the output color correction matrix.
    /**
     *  This property will store the first row of a user defined 3x3 output color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::ctocmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistOutputCorrectionMatrixRow0;
    /// \brief The second row of the output color correction matrix.
    /**
     *  This property will store the second row of a user defined 3x3 output color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::ctocmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistOutputCorrectionMatrixRow1;
    /// \brief The third row of the output color correction matrix.
    /**
     *  This property will store the third row of a user defined 3x3 output color correction matrix if
     *  \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode is set to
     *  \b mvIMPACT::acquire::ctocmmUser. For other values of \b mvIMPACT::acquire::ImageProcessing::colorTwistOutputCorrectionMatrixMode
     *  modifying this property will have no effect.
     */
    PropertyF colorTwistOutputCorrectionMatrixRow2;
    /// \brief The first row of the resulting color twist matrix.
    /**
     *  This property will store the first row of the 3x4 resulting color twist matrix. This matrix
     *  is created by multiplying the input correction matrix by the color twist matrix by the output correction matrix.
     *  Only active matrices will be used to calculate the resulting matrix.
     */
    PropertyF colorTwistResultingMatrixRow0;
    /// \brief The second row of the resulting color twist matrix.
    /**
     *  This property will store the second row of the 3x4 resulting color twist matrix. This matrix
     *  is created by multiplying the input correction matrix by the color twist matrix by the output correction matrix.
     *  Only active matrices will be used to calculate the resulting matrix.
     */
    PropertyF colorTwistResultingMatrixRow1;
    /// \brief The third row of the resulting color twist matrix.
    /**
     *  This property will store the third row of the 3x4 resulting color twist matrix. This matrix
     *  is created by multiplying the input correction matrix by the color twist matrix by the output correction matrix.
     *  Only active matrices will be used to calculate the resulting matrix.
     */
    PropertyF colorTwistResultingMatrixRow2;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIColorProcessingMode getColorProcessing( void ) const
    {
        return colorProcessing;
    }
    PropertyIBayerConversionMode getBayerConversionMode( void ) const
    {
        return bayerConversionMode;
    }
    PropertyIWhiteBalanceParameter getWhiteBalance( void ) const
    {
        return whiteBalance;
    }
    PropertyIWhiteBalanceCalibrationMode getWhiteBalanceCalibration( void ) const
    {
        return whiteBalanceCalibration;
    }
    PropertyIImageProcessingFilter getFilter( void ) const
    {
        return filter;
    }
    PropertyIMirrorOperationMode getMirrorOperationMode( void ) const
    {
        return mirrorOperationMode;
    }
    PropertyIMirrorMode getMirrorModeGlobal( void ) const
    {
        return mirrorModeGlobal;
    }
    PropertyIDefectivePixelsFilterMode getDefectivePixelsFilterMode( void ) const
    {
        return defectivePixelsFilterMode;
    }
    PropertyI getDefectivePixelsFilterLeakyPixelDeviation_ADCLimit( void ) const
    {
        return defectivePixelsFilterLeakyPixelDeviation_ADCLimit;
    }
    PropertyI getDectivePixelsFilterColdPixelDeviation_pc( void ) const
    {
        return defectivePixelsFilterColdPixelDeviation_pc;
    }
    PropertyI getDefectivePixelsFound( void ) const
    {
        return defectivePixelsFound;
    }
    PropertyIFlatFieldFilterMode getFlatFieldFilterMode( void ) const
    {
        return flatFieldFilterMode;
    }
    PropertyI getFlatFieldFilterCalibrationImageCount( void ) const
    {
        return flatFieldFilterCalibrationImageCount;
    }
    PropertyIDarkCurrentFilterMode getDarkCurrentFilterMode( void ) const
    {
        return darkCurrentFilterMode;
    }
    PropertyI getDarkCurrentFilterCalibrationImageCount( void ) const
    {
        return darkCurrentFilterCalibrationImageCount;
    }
    PropertyIBoolean getGainOffsetKneeEnable( void ) const
    {
        return gainOffsetKneeEnable;
    }
    PropertyF getGainOffsetKneeMasterOffset_pc( void ) const
    {
        return gainOffsetKneeMasterOffset_pc;
    }
    PropertyIBoolean getLUTEnable( void ) const
    {
        return LUTEnable;
    }
    PropertyILUTMode getLUTMode( void ) const
    {
        return LUTMode;
    }
    PropertyILUTInterpolationMode getLUTInterpolationMode( void ) const
    {
        return LUTInterpolationMode;
    }
    PropertyILUTImplementation getLUTImplementation( void ) const
    {
        return LUTImplementation;
    }
    PropertyILUTMapping getLUTMappingHardware( void ) const
    {
        return LUTMappingHardware;
    }
    PropertyILUTMapping getLUTMappingSoftware( void ) const
    {
        return LUTMappingSoftware;
    }
    PropertyIBoolean getTapSortEnable( void ) const
    {
        return tapSortEnable;
    }
    PropertyIBoolean getChannelSplitEnable( void ) const
    {
        return channelSplitEnable;
    }
    PropertyIChannelSplitMode getChannelSplitMode( void ) const
    {
        return channelSplitMode;
    }
    PropertyI getChannelSplitChannelIndex( void ) const
    {
        return channelSplitChannelIndex;
    }
    PropertyIBoolean getChannelSplitDeinterlaceEnable( void ) const
    {
        return channelSplitDeinterlaceEnable;
    }
    PropertyIBoolean getColorTwistInputCorrectionMatrixEnable( void ) const
    {
        return colorTwistInputCorrectionMatrixEnable;
    }
    PropertyIColorTwistInputCorrectionMatrixMode getColorTwistInputCorrectionMatrixMode( void ) const
    {
        return colorTwistInputCorrectionMatrixMode;
    }
    PropertyF getColorTwistInputCorrectionMatrixRow0( void ) const
    {
        return colorTwistInputCorrectionMatrixRow0;
    }
    PropertyF getColorTwistInputCorrectionMatrixRow1( void ) const
    {
        return colorTwistInputCorrectionMatrixRow1;
    }
    PropertyF getColorTwistInputCorrectionMatrixRow2( void ) const
    {
        return colorTwistInputCorrectionMatrixRow2;
    }
    PropertyIBoolean getColorTwistEnable( void ) const
    {
        return colorTwistEnable;
    }
    PropertyF getColorTwistRow0( void ) const
    {
        return colorTwistRow0;
    }
    PropertyF getColorTwistRow1( void ) const
    {
        return colorTwistRow1;
    }
    PropertyF getColorTwistRow2( void ) const
    {
        return colorTwistRow2;
    }
    PropertyIBoolean getColorTwistOutputCorrectionMatrixEnable( void ) const
    {
        return colorTwistOutputCorrectionMatrixEnable;
    }
    PropertyIColorTwistOutputCorrectionMatrixMode getColorTwistOutputCorrectionMatrixMode( void ) const
    {
        return colorTwistOutputCorrectionMatrixMode;
    }
    PropertyF getColorTwistOutputCorrectionMatrixRow0( void ) const
    {
        return colorTwistOutputCorrectionMatrixRow0;
    }
    PropertyF getColorTwistOutputCorrectionMatrixRow1( void ) const
    {
        return colorTwistOutputCorrectionMatrixRow1;
    }
    PropertyF getColorTwistOutputCorrectionMatrixRow2( void ) const
    {
        return colorTwistOutputCorrectionMatrixRow2;
    }
    PropertyF getColorTwistResultingMatrixRow0( void ) const
    {
        return colorTwistResultingMatrixRow0;
    }
    PropertyF getColorTwistResultingMatrixRow1( void ) const
    {
        return colorTwistResultingMatrixRow1;
    }
    PropertyF getColorTwistResultingMatrixRow2( void ) const
    {
        return colorTwistResultingMatrixRow2;
    }
#endif // #ifdef DOTNET_ONLY_CODE
    /// \brief Sets the saturation by using the color twist matrix.
    /**
     *  The following saturation formula is used:
     *
     * \code
     *   [0.299 + 0.701*K       , 0.587*(1-K) , 0.114*(1-K)     ]
     *   [0.299*(1-K) + 0.701*K , 0.587       , 0.114*(1-K)     ]
     *   [0.299*(1-K) + 0.701*K , 0.587*(1-K) , 0.114 + 0.886*K ]
     * \endcode
     *
     *  K is the saturation factor
     *  K > 1 increases saturation
     *  K = 1 means no change
     *  0 < K < 1 decreases saturation
     *  K = 0 produces B&W
     *  K < 0 inverts color
     *
     *  \note
     *  To enable/disable the saturation the application must write to \b mvIMPACT::acquire::ImageProcessing::colorTwistEnable.
     */
    void setSaturation(
        /// [in] The saturation value.
        double K )
    {
        std::vector<double> row( 3, 0. );
        row[0] = 0.299       + 0.701 * K;
        row[1] = 0.587 * ( 1 - K );
        row[2] = 0.114 * ( 1 - K );
        colorTwistRow0.write( row, true );
        row[0] = 0.299 * ( 1 - K );
        row[1] = 0.587       + 0.413 * K;
        row[2] = 0.114 * ( 1 - K );
        colorTwistRow1.write( row, true );
        row[0] = 0.299 * ( 1 - K );
        row[1] = 0.587 * ( 1 - K );
        row[2] = 0.114       + 0.886 * K;
        colorTwistRow2.write( row, true );
    }
    /// \brief Returns a reference to a set of user definable parameters to configure a certain channel of the GainOffsetKnee filter.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A reference to one of the parameter sets for the GainOffsetKnee filter that
     *  can be altered by the user.
     */
    GainOffsetKneeChannelParameters& getGainOffsetKneeParameter(
        /// [in] The index of the gain, offset, knee parameter set to be returned
        unsigned int index ) const
    {
        return *( m_pRefData->m_GainOffsetKneeParameters.at( index ) );
    }
    /// \brief Returns the number of available user definable parameter sets for the GainOffsetKnee filter.
    unsigned int getGainOffsetKneeParameterCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_GainOffsetKneeParameters.size() );
    }
    /// \brief Returns a reference to a set of user definable parameters to configure mirror filter related features.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A reference to one of the parameter sets for mirror filter related features that
     *  can be altered by the user.
     */
    MirrorParameters& getMirrorParameter(
        /// [in] The index of the mirror parameter set to be returned
        unsigned int index ) const
    {
        return *( m_pRefData->m_MirrorParameters.at( index ) );
    }
    /// \brief Returns the number of available user definable parameter sets mirror filter related operations.
    unsigned int getMirrorParameterCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_MirrorParameters.size() );
    }
    /// \brief Returns a reference to a set of user definable parameters to configure LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) operations.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A reference to one of the parameter sets for LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) operations that
     *  can be altered by the user.
     */
    LUTParameters& getLUTParameter(
        /// [in] The index of the LUT parameter set to be returned
        unsigned int index ) const
    {
        return *( m_pRefData->m_LUTParameters.at( index ) );
    }
    /// \brief Returns the number of available user definable parameter sets for LUT (<b>L</b>ook <b>U</b>p <b>T</b>able) operations.
    unsigned int getLUTParameterCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_LUTParameters.size() );
    }
    /// \brief Returns a reference to a set of user definable parameters to perform a white balance correction.
    /**
     *  Pass '0' for the setting that is activated by setting the property \b mvIMPACT::acquire::ImageProcessing::whiteBalance
     *  to \b mvIMPACT::acquire::wbpUser1, '1' for
     *  \b mvIMPACT::acquire::wbpUser2 and so on.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A reference to one of the parameter sets for white balance correction that
     *  can be altered by the user.
     */
    WhiteBalanceSettings& getWBUserSetting(
        /// [in] The index of the white balance parameter set to be returned
        unsigned int index ) const
    {
        return *( m_pRefData->m_WBUserSettings.at( index ) );
    }
    /// \brief Returns the number of available user definable parameter sets for white balance correction.
    unsigned int getWBUserSettingsCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_WBUserSettings.size() );
    }
};

//-----------------------------------------------------------------------------
/// \brief Contains basic statistical information
/**
 *  This class contains basic statistical information about the \b mvIMPACT::acquire::Device and the
 *  the current image acquisition progress.
 */
class StatisticsBase : public ComponentCollection
//-----------------------------------------------------------------------------
{
    Method m_resetStatistics;
public:
    explicit StatisticsBase(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : ComponentCollection( pDev ), m_resetStatistics(), captureTime_s(),
        errorCount(), abortedRequestsCount(), timedOutRequestsCount(), framesPerSecond(),
        frameCount(), imageProcTime_s(), formatConvertTime_s(), queueTime_s(), lostImagesCount(),
        framesIncompleteCount(), missingDataAverage_pc(), retransmitCount()
    {
        DeviceComponentLocator locator( pDev, dltStatistics );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( m_resetStatistics, "ResetStatistics@i" );
        locator.bindComponent( captureTime_s, "CaptureTime_s" );
        locator.bindComponent( errorCount, "ErrorCount" );
        locator.bindComponent( abortedRequestsCount, "AbortedRequestsCount" );
        locator.bindComponent( timedOutRequestsCount, "TimedOutRequestsCount" );
        locator.bindComponent( framesPerSecond, "FramesPerSecond" );
        locator.bindComponent( frameCount, "FrameCount" );
        locator.bindComponent( imageProcTime_s, "ImageProcTime_s" );
        locator.bindComponent( formatConvertTime_s, "FormatConvertTime_s" );
        locator.bindComponent( queueTime_s, "QueueTime_s" );
        locator.bindComponent( lostImagesCount, "LostImagesCount" );
        locator.bindComponent( framesIncompleteCount, "FramesIncompleteCount" );
        locator.bindComponent( missingDataAverage_pc, "MissingDataAverage_pc" );
        locator.bindComponent( retransmitCount, "RetransmitCount" );
    }
    /// \brief Resets all statistical properties.
    /**
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int reset( void ) const
    {
        if( !m_resetStatistics.isValid() )
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
        return m_resetStatistics.call();
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A float property \b (read-only) containing the average time needed to capture an image.
    /**
     *  This time is \b NOT the reciprocal of the frame rate, but usually a longer period of time as it includes the time
     *  elapsed from the trigger signal to the end of the image processing time. Typically more images can be captured
     *  as the processing can run in parallel to the sensor readout for example.
     */
    PropertyF captureTime_s;
    /// \brief An integer property \b (read-only) containing the overall count of image requests which returned with an error since the
    /// \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    PropertyI errorCount;
    /// \brief An integer property \b (read-only) containing the overall count of image requests which have been aborted since the
    /// \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    PropertyI abortedRequestsCount;
    /// \brief An integer property \b (read-only) containing the overall count of image requests which have timed out since the
    /// \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    PropertyI timedOutRequestsCount;
    /// \brief A float property \b (read-only) containing the current number of frames captured per second.
    PropertyF framesPerSecond;
    /// \brief An integer property \b (read-only) containing the overall count of images captured since the
    /// \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    PropertyI frameCount;
    /// \brief An integer property \b (read-only) containing the time (in seconds) needed to process the image data internally.
    /**
     *  This time might be interesting e.g. if the image data is transformed into a color
     *  image by the driver (e.g. when working with Bayer mosaic cameras) or if any other processing function is applied to the
     *  image before it is transferred to the user.
     */
    PropertyF imageProcTime_s;
    /// \brief An integer property \b (read-only) containing the time (in seconds) needed to convert the image data delivered by the hardware and/or active internal driver filter nodes to the format specified by \b mvIMPACT::acquire::ImageDestination::pixelFormat internally.
    /**
     *  This time will be almost 0 when no custom destination pixel format has been specified and if a custom destination format
     *  was selected and this format differs from the pixel format that is transferred by the hardware this time will be a part
     *  (or all, but never more) than the time reported by \b mvIMPACT::acquire::Statistics::imageProcTime_s.
     */
    PropertyF formatConvertTime_s;
    /// \brief A float property \b (read-only) containing the average time (in seconds) a \b mvIMPACT::acquire::Request object spends in the request queue of the device.
    PropertyF queueTime_s;
    /// \brief An integer property \b (read-only) containing the number of images that have been lost from a continuous image stream coming from a video signal source.
    /**
     *  \note
     *  For some devices this value is based on an assumption so it's by no means absolutely accurate, but when this
     *  value starts to increase this might indicate that a problem is present.
     *
     *  Possible causes (among others) might be:
     *  - the capture device or device driver can't cope with the amount of data the video signal source is delivering in the current mode of operation
     *  - the capture device or device driver can't transfer all the data to the host PC because of bus bandwidth shortages
     *  - the capture device or device driver image request queue ran low because the system is busy doing other things or because of a bug
     *  in the application
     *
     *  \note
     *  This feature is supported by every device driver. If a device does not seem to support this feature (calling \b mvIMPACT::acquire::Component::isValid returns false)
     *  a driver update will fix this.
     */
    PropertyI lostImagesCount;
    /// \brief An integer property \b (read-only) containing the overall count of image requests which haven't been captured completely since the \b mvIMPACT::acquire::Device has been opened.
    /**
     *  This value is a good indicator for bandwidth problems on the device to host connection.
     *
     *  \note
     *  This feature is supported by every device driver. If a device does not seem to support this feature (calling \b mvIMPACT::acquire::Component::isValid returns false)
     *  a driver update will fix this.
     */
    PropertyI framesIncompleteCount;
    /// \brief A float property \b (read-only) containing the average amount of data missing in frames that haven't been captured completely since the \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    /**
     *  This value is a good indicator for bandwidth problems on the device to host connection.
     *
     *  \note
     *  This feature is supported by every device driver. If a device does not seem to support this feature (calling \b mvIMPACT::acquire::Component::isValid returns false)
     *  a driver update will fix this.
     */
    PropertyF missingDataAverage_pc;
    /// \brief A 64 bit integer property \b (read-only) containing the number of retransmit requests issued due to temporary bus bandwidth shortages or other transmission problems since this \b mvIMPACT::acquire::Device has been opened OR since the last call to \b mvIMPACT::acquire::StatisticsBase::reset().
    /**
     *  This value can be an indicator for transmission problems because of bad signal quality or bus
     *  bandwidth shortages in the system.
     *  Ideally this property should stay 0 all the time if the system can cope with the amount of data transferred and the signal
     *  quality is good. However it might increase slowly and still every frame will be flawless as the driver internally was
     *  able to restore the lost data by re-requesting it from the video signal source again.
     *
     *  \note
     *  This feature is supported by every device driver. If a device does not seem to support this feature (calling \b mvIMPACT::acquire::Component::isValid returns false)
     *  a driver update will fix this.
     */
    PropertyI64 retransmitCount;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyF getCaptureTime_s( void ) const
    {
        return captureTime_s;
    }
    PropertyI getErrorCount( void ) const
    {
        return errorCount;
    }
    PropertyI getAbortedRequestsCount( void ) const
    {
        return abortedRequestsCount;
    }
    PropertyI getTimedOutRequestsCount( void ) const
    {
        return timedOutRequestsCount;
    }
    PropertyF getFramesPerSecond( void ) const
    {
        return framesPerSecond;
    }
    PropertyI getFrameCount( void ) const
    {
        return frameCount;
    }
    PropertyF getImageProcTime_s( void ) const
    {
        return imageProcTime_s;
    }
    PropertyF getFormatConvertTime_s( void ) const
    {
        return formatConvertTime_s;
    }
    PropertyF getQueueTime_s( void ) const
    {
        return queueTime_s;
    }
    PropertyI getLostImagesCount( void ) const
    {
        return lostImagesCount;
    }
    PropertyI getFramesIncompleteCount( void ) const
    {
        return framesIncompleteCount;
    }
    PropertyF getMissingDataAverage_pc( void ) const
    {
        return missingDataAverage_pc;
    }
    PropertyI64 getRetransmitCount( void ) const
    {
        return retransmitCount;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains statistical information
/**
 *  This class contains general statistical information about the \b mvIMPACT::acquire::Device and the
 *  the current image acquisition progress.
 */
class Statistics : public StatisticsBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::Statistics object.
    explicit Statistics(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : StatisticsBase( pDev ) {}
};

//-----------------------------------------------------------------------------
/// \brief A base class for accessing settings that control the overall behaviour of a device driver.
/**
 *  Instances of this class can't be constructed directly. Use one of the derived types.
 */
class SystemBase : public ComponentCollection
//-----------------------------------------------------------------------------
{
protected:
    /// \brief Constructs a new \b mvIMPACT::acquire::SystemBase object.
    explicit SystemBase(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : ComponentCollection( pDev ), requestCount(), workerPriority(),
        acquisitionMode(), acquisitionIdleTimeMax_ms()
    {
        DeviceComponentLocator locator( pDev, dltSystemSettings );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( requestCount, "RequestCount" );
        locator.bindComponent( workerPriority, "WorkerPriority" );
        locator.bindComponent( acquisitionMode, "AcquisitionMode" );
        locator.bindComponent( acquisitionIdleTimeMax_ms, "AcquisitionIdleTimeMax_ms" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the number of requests allocated by the driver.
    /**
     *  Each request object can be used to capture data into. Multiple requests can be processed
     *  by a device driver as background tasks.
     *
     *  \note for performance reasons this value can be increased at any time(only exception: Working with the interface layout
     *  \b mvIMPACT::acquire::dilGenICam or \b mvIMPACT::acquire::dilGeneric while streaming is active) but can only be
     *  decreased when no request object is currently locked by the application and no requests has been
     *  queued for acquisition. So modifying this value might require the application to call \b mvIMPACT::acquire::FunctionInterface::imageReqestReset
     *  and several calls to \b mvIMPACT::acquire::Request::unlock before.
     *
     *  There are not too many reasons to modify the default request count suggested by the device driver. Good reasons include:
     *
     *  - a single image is huge compared to the overall system memory(e.g. a single image has 200MB while the system itself only
     *  has about 2GB of RAM. Here it might make sense to reduce the number of capture buffers to 1 or 2.
     *  - the frame rate is high(larger then 100 frames per second) and no frames shall be lost. Here it might make sense to set
     *  the number of capture buffers to something like framerate divided by 10 as a rule of thumb.
     *
     *  \sa
     *  \b mvIMPACT::acquire::FunctionInterface::requestCount
     */
    PropertyI requestCount;
    /// \brief An enumerated integer property defining the thread priority of the drivers internal worker thread.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TThreadPriority.
     */
    PropertyIThreadPriority workerPriority;
    /// \brief An enumerated integer property defining the acquisition mode of the device.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAcquisitionMode.
     */
    PropertyIAcquisitionMode acquisitionMode;
    /// \brief An integer property defining the maximum time in ms the driver waits for new requests being queued until streaming by the device will automatically be stopped.
    /**
     *  This property defines the maximum time in milliseconds the driver waits for new \b mvIMPACT::acquire::Request objects being
     *  queued by calling \b mvIMPACT::acquire::FunctionInterface::imageRequestSingle once the driver will automatically send an
     *  acquisition stop command to a streaming device (such as a GigE Vision device) when the driver's request queue has run empty (thus the
     *  driver has no more buffers to capture data into).
     *
     *  This property will only be taken into account when \b mvIMPACT::acquire::Device::acquisitionStartStopBehaviour has been set to
     *  \b mvIMPACT::acquire::assbDefault.
     *
     *  \note
     *  This property will not be available for every device. Right now only devices operated through the \a GenTL driver package will support
     *  this feature.
     */
    PropertyI acquisitionIdleTimeMax_ms;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyI getRequestCount( void ) const
    {
        return requestCount;
    }
    PropertyIThreadPriority getWorkerPriority( void ) const
    {
        return workerPriority;
    }
    PropertyIAcquisitionMode getAcquisitionMode( void ) const
    {
        return acquisitionMode;
    }
    PropertyI getAcquisitionIdleTimeMax_ms( void ) const
    {
        return acquisitionIdleTimeMax_ms;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class for accessing general settings that control the overall behaviour of a device driver.
class SystemSettings : public SystemBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::SystemSettings object.
    explicit SystemSettings( /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : SystemBase( pDev ) {}
};

/// @}

#ifndef IGNORE_MVDEVICE_SPECIFIC_INTERFACE_DOCUMENTATION

/// \defgroup DeviceSpecificInterface Device specific interface layout
/// \brief Classes and functions that will be available if the device is used in \a device \a specific interface layout.
///
/// This group contains classes and functions that will be available if the device is used
/// in \b mvIMPACT::acquire::dilDeviceSpecific interface layout.
///
/// @{

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAcquisitionField
typedef EnumPropertyI<TAcquisitionField> PropertyIAcquisitionField;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAcquisitionField, EnumPropertyI, mvIMPACT::acquire::TAcquisitionField ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAutoControlSpeed
typedef EnumPropertyI<TAutoControlSpeed> PropertyIAutoControlSpeed;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAutoControlSpeed, EnumPropertyI, mvIMPACT::acquire::TAutoControlSpeed ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAutoControlMode
typedef EnumPropertyI<TAutoControlMode> PropertyIAutoControlMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAutoControlMode, EnumPropertyI, mvIMPACT::acquire::TAutoControlMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAutoExposureControl
typedef EnumPropertyI<TAutoExposureControl> PropertyIAutoExposureControl;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAutoExposureControl, EnumPropertyI, mvIMPACT::acquire::TAutoExposureControl ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAutoGainControl
typedef EnumPropertyI<TAutoGainControl> PropertyIAutoGainControl;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAutoGainControl, EnumPropertyI, mvIMPACT::acquire::TAutoGainControl ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TAutoOffsetCalibration
typedef EnumPropertyI<TAutoOffsetCalibration> PropertyIAutoOffsetCalibration;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIAutoOffsetCalibration, EnumPropertyI, mvIMPACT::acquire::TAutoOffsetCalibration ) )

#ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXDigitalInputThreshold
typedef EnumPropertyI<TBlueFOXDigitalInputThreshold> PropertyIBlueFOXDigitalInputThreshold;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXDigitalInputThreshold, EnumPropertyI, mvIMPACT::acquire::TBlueFOXDigitalInputThreshold ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXFooterMode
typedef EnumPropertyI<TBlueFOXFooterMode> PropertyIBlueFOXFooterMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXFooterMode, EnumPropertyI, mvIMPACT::acquire::TBlueFOXFooterMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXInfoSensorCapabilities
typedef EnumPropertyI<TBlueFOXInfoSensorCapabilities> PropertyIBlueFOXInfoSensorCapabilities;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXInfoSensorCapabilities, EnumPropertyI, mvIMPACT::acquire::TBlueFOXInfoSensorCapabilities ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXOffsetAutoBlackSpeed
typedef EnumPropertyI<TBlueFOXOffsetAutoBlackSpeed> PropertyIBlueFOXOffsetAutoBlackSpeed;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXOffsetAutoBlackSpeed, EnumPropertyI, mvIMPACT::acquire::TBlueFOXOffsetAutoBlackSpeed ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXSensorTiming
typedef EnumPropertyI<TBlueFOXSensorTiming> PropertyIBlueFOXSensorTiming;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXSensorTiming, EnumPropertyI, mvIMPACT::acquire::TBlueFOXSensorTiming ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TBlueFOXTransferSize
typedef EnumPropertyI<TBlueFOXTransferSize> PropertyIBlueFOXTransferSize;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIBlueFOXTransferSize, EnumPropertyI, mvIMPACT::acquire::TBlueFOXTransferSize ) )
#endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraAoiMode
typedef EnumPropertyI<TCameraAoiMode> PropertyICameraAoiMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraAoiMode, EnumPropertyI, mvIMPACT::acquire::TCameraAoiMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraBinningMode
typedef EnumPropertyI<TCameraBinningMode> PropertyICameraBinningMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraBinningMode, EnumPropertyI, mvIMPACT::acquire::TCameraBinningMode ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraDataFormat
typedef EnumPropertyI<TCameraDataFormat> PropertyICameraDataFormat;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraDataFormat, EnumPropertyI, mvIMPACT::acquire::TCameraDataFormat ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraExposeMode
typedef EnumPropertyI<TCameraExposeMode> PropertyICameraExposeMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraExposeMode, EnumPropertyI, mvIMPACT::acquire::TCameraExposeMode ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraExternalSyncEdge
typedef EnumPropertyI<TCameraExternalSyncEdge> PropertyICameraExternalSyncEdge;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraExternalSyncEdge, EnumPropertyI, mvIMPACT::acquire::TCameraExternalSyncEdge ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraExternalSyncOutput
typedef EnumPropertyI<TCameraExternalSyncOutput> PropertyICameraExternalSyncOutput;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraExternalSyncOutput, EnumPropertyI, mvIMPACT::acquire::TCameraExternalSyncOutput ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraFlashMode
typedef EnumPropertyI<TCameraFlashMode> PropertyICameraFlashMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraFlashMode, EnumPropertyI, mvIMPACT::acquire::TCameraFlashMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraFlashType
typedef EnumPropertyI<TCameraFlashType> PropertyICameraFlashType;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraFlashType, EnumPropertyI, mvIMPACT::acquire::TCameraFlashType ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraHDRMode
typedef EnumPropertyI<TCameraHDRMode> PropertyICameraHDRMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraHDRMode, EnumPropertyI, mvIMPACT::acquire::TCameraHDRMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraInterlacedType
typedef EnumPropertyI<TCameraInterlacedType> PropertyICameraInterlacedType;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraInterlacedType, EnumPropertyI, mvIMPACT::acquire::TCameraInterlacedType ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraLinkDataValidMode
typedef EnumPropertyI<TCameraLinkDataValidMode> PropertyICameraLinkDataValidMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraLinkDataValidMode, EnumPropertyI, mvIMPACT::acquire::TCameraLinkDataValidMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraPixelClock
typedef EnumPropertyI<TCameraPixelClock> PropertyICameraPixelClock;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraPixelClock, EnumPropertyI, mvIMPACT::acquire::TCameraPixelClock ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraScanMode
typedef EnumPropertyI<TCameraScanMode> PropertyICameraScanMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraScanMode, EnumPropertyI, mvIMPACT::acquire::TCameraScanMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraShutterMode
typedef EnumPropertyI<TCameraShutterMode> PropertyICameraShutterMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraShutterMode, EnumPropertyI, mvIMPACT::acquire::TCameraShutterMode ) )
#endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraTapsXGeometry
typedef EnumPropertyI<TCameraTapsXGeometry> PropertyICameraTapsXGeometry;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraTapsXGeometry, EnumPropertyI, mvIMPACT::acquire::TCameraTapsXGeometry ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraTapsYGeometry
typedef EnumPropertyI<TCameraTapsYGeometry> PropertyICameraTapsYGeometry;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraTapsYGeometry, EnumPropertyI, mvIMPACT::acquire::TCameraTapsYGeometry ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraTriggerMode
typedef EnumPropertyI<TCameraTriggerMode> PropertyICameraTriggerMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraTriggerMode, EnumPropertyI, mvIMPACT::acquire::TCameraTriggerMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraTestMode
typedef EnumPropertyI<TCameraTestMode> PropertyICameraTestMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraTestMode, EnumPropertyI, mvIMPACT::acquire::TCameraTestMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraSerialPortBaudRate
typedef EnumPropertyI<TCameraSerialPortBaudRate> PropertyICameraSerialPortBaudRate;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraSerialPortBaudRate, EnumPropertyI, mvIMPACT::acquire::TCameraSerialPortBaudRate ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TCameraTriggerSource
typedef EnumPropertyI<TCameraTriggerSource> PropertyICameraTriggerSource;
PYTHON_ONLY( ENUM_PROPERTY( PropertyICameraTriggerSource, EnumPropertyI, mvIMPACT::acquire::TCameraTriggerSource ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TClampMode
typedef EnumPropertyI<TClampMode> PropertyIClampMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIClampMode, EnumPropertyI, mvIMPACT::acquire::TClampMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceAdvancedOptions
typedef EnumPropertyI<TDeviceAdvancedOptions> PropertyIDeviceAdvancedOptions;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceAdvancedOptions, EnumPropertyI, mvIMPACT::acquire::TDeviceAdvancedOptions ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceDigitalOutputMode
typedef EnumPropertyI<TDeviceDigitalOutputMode> PropertyIDeviceDigitalOutputMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceDigitalOutputMode, EnumPropertyI, mvIMPACT::acquire::TDeviceDigitalOutputMode ) )
#endif // IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceEventMode
typedef EnumPropertyI<TDeviceEventMode> PropertyIDeviceEventMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceEventMode, EnumPropertyI, mvIMPACT::acquire::TDeviceEventMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceEventType
typedef EnumPropertyI<TDeviceEventType> PropertyIDeviceEventType;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceEventType, EnumPropertyI, mvIMPACT::acquire::TDeviceEventType ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceImageTrigger
typedef EnumPropertyI<TDeviceImageTrigger> PropertyIDeviceImageTrigger;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceImageTrigger, EnumPropertyI, mvIMPACT::acquire::TDeviceImageTrigger ) )
#endif // IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDevicePowerMode
typedef EnumPropertyI<TDevicePowerMode> PropertyIDevicePowerMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDevicePowerMode, EnumPropertyI, mvIMPACT::acquire::TDevicePowerMode ) )
#endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceScanRateMode
typedef EnumPropertyI<TDeviceScanRateMode> PropertyIDeviceScanRateMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceScanRateMode, EnumPropertyI, mvIMPACT::acquire::TDeviceScanRateMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceSignalOutputStartEvent
typedef EnumPropertyI<TDeviceSignalOutputStartEvent> PropertyIDeviceSignalOutputStartEvent;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceSignalOutputStartEvent, EnumPropertyI, mvIMPACT::acquire::TDeviceSignalOutputStartEvent ) )
#endif // IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceSyncOutMode
typedef EnumPropertyI<TDeviceSyncOutMode> PropertyIDeviceSyncOutMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceSyncOutMode, EnumPropertyI, mvIMPACT::acquire::TDeviceSyncOutMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceTriggerInterface
typedef EnumPropertyI<TDeviceTriggerInterface> PropertyIDeviceTriggerInterface;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceTriggerInterface, EnumPropertyI, mvIMPACT::acquire::TDeviceTriggerInterface ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDeviceTriggerMode
typedef EnumPropertyI<TDeviceTriggerMode> PropertyIDeviceTriggerMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDeviceTriggerMode, EnumPropertyI, mvIMPACT::acquire::TDeviceTriggerMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDigIOState
typedef EnumPropertyI<TDigIOState> PropertyIDigIOState;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDigIOState, EnumPropertyI, mvIMPACT::acquire::TDigIOState ) )

#ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDigitalIOMeasurementMode
typedef EnumPropertyI<TDigitalIOMeasurementMode> PropertyIDigitalIOMeasurementMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDigitalIOMeasurementMode, EnumPropertyI, mvIMPACT::acquire::TDigitalIOMeasurementMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDigitalIOMeasurementSource
typedef EnumPropertyI<TDigitalIOMeasurementSource> PropertyIDigitalIOMeasurementSource;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDigitalIOMeasurementSource, EnumPropertyI, mvIMPACT::acquire::TDigitalIOMeasurementSource ) )
#endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDigitalOutputControlMode
typedef EnumPropertyI<TDigitalOutputControlMode> PropertyIDigitalOutputControlMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDigitalOutputControlMode, EnumPropertyI, mvIMPACT::acquire::TDigitalOutputControlMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TDigitalSignal
typedef EnumPropertyI<TDigitalSignal> PropertyIDigitalSignal;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIDigitalSignal, EnumPropertyI, mvIMPACT::acquire::TDigitalSignal ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TFieldGateMode
typedef EnumPropertyI<TFieldGateMode> PropertyIFieldGateMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIFieldGateMode, EnumPropertyI, mvIMPACT::acquire::TFieldGateMode ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TI2COperationMode
typedef EnumPropertyI<TI2COperationMode> PropertyII2COperationMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyII2COperationMode, EnumPropertyI, mvIMPACT::acquire::TI2COperationMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TI2COperationStatus
typedef EnumPropertyI<TI2COperationStatus> PropertyII2COperationStatus;
PYTHON_ONLY( ENUM_PROPERTY( PropertyII2COperationStatus, EnumPropertyI, mvIMPACT::acquire::TI2COperationStatus ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TInfoSensorColorMode
typedef EnumPropertyI<TInfoSensorColorMode> PropertyIInfoSensorColorMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIInfoSensorColorMode, EnumPropertyI, mvIMPACT::acquire::TInfoSensorColorMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TInfoSensorType
typedef EnumPropertyI<TInfoSensorType> PropertyIInfoSensorType;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIInfoSensorType, EnumPropertyI, mvIMPACT::acquire::TInfoSensorType ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TInterlacedMode
typedef EnumPropertyI<TInterlacedMode> PropertyIInterlacedMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIInterlacedMode, EnumPropertyI, mvIMPACT::acquire::TInterlacedMode ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TLineCounter
typedef EnumPropertyI<TLineCounter> PropertyILineCounter;
PYTHON_ONLY( ENUM_PROPERTY( PropertyILineCounter, EnumPropertyI, mvIMPACT::acquire::TLineCounter ) )
#endif // IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#if !defined(IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TMemoryManagerMode
typedef EnumPropertyI<TMemoryManagerMode> PropertyIMemoryManagerMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIMemoryManagerMode, EnumPropertyI, mvIMPACT::acquire::TMemoryManagerMode ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TMemoryManagerPoolMode
typedef EnumPropertyI<TMemoryManagerPoolMode> PropertyIMemoryManagerPoolMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIMemoryManagerPoolMode, EnumPropertyI, mvIMPACT::acquire::TMemoryManagerPoolMode ) )
#endif // #if !defined(IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TOnBoardMemoryMode
typedef EnumPropertyI<TOnBoardMemoryMode> PropertyIOnBoardMemoryMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIOnBoardMemoryMode, EnumPropertyI, mvIMPACT::acquire::TOnBoardMemoryMode ) )
#endif // IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TPulseStartTrigger
typedef EnumPropertyI<TPulseStartTrigger> PropertyIPulseStartTrigger;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIPulseStartTrigger, EnumPropertyI, mvIMPACT::acquire::TPulseStartTrigger ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TRTCtrlModes
typedef EnumPropertyI<TRTCtrlModes> PropertyIRTCtrlModes;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIRTCtrlModes, EnumPropertyI, mvIMPACT::acquire::TRTCtrlModes ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TRTProgOpCodes
typedef EnumPropertyI<TRTProgOpCodes> PropertyIRTProgOpCodes;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIRTProgOpCodes, EnumPropertyI, mvIMPACT::acquire::TRTProgOpCodes ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TScanClock
typedef EnumPropertyI<TScanClock> PropertyIScanClock;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIScanClock, EnumPropertyI, mvIMPACT::acquire::TScanClock ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TScanStandard
typedef EnumPropertyI<TScanStandard> PropertyIScanStandard;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIScanStandard, EnumPropertyI, mvIMPACT::acquire::TScanStandard ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TTriggerMoment
typedef EnumPropertyI<TTriggerMoment> PropertyITriggerMoment;
PYTHON_ONLY( ENUM_PROPERTY( PropertyITriggerMoment, EnumPropertyI, mvIMPACT::acquire::TTriggerMoment ) )

#ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TVideoStandard
typedef EnumPropertyI<TVideoStandard> PropertyIVideoStandard;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIVideoStandard, EnumPropertyI, mvIMPACT::acquire::TVideoStandard ) )
#endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#ifndef IGNORE_MVVIRTUALDEVICE_SPECIFIC_DOCUMENTATION
/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TVirtualDeviceImageType
typedef EnumPropertyI<TVirtualDeviceImageType> PropertyIVirtualDeviceImageType;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIVirtualDeviceImageType, EnumPropertyI, mvIMPACT::acquire::TVirtualDeviceImageType ) )

/// \brief Defines a property for values defined by \b mvIMPACT::acquire::TVirtualDeviceTestMode
typedef EnumPropertyI<TVirtualDeviceTestMode> PropertyIVirtualDeviceTestMode;
PYTHON_ONLY( ENUM_PROPERTY( PropertyIVirtualDeviceTestMode, EnumPropertyI, mvIMPACT::acquire::TVirtualDeviceTestMode ) )
#endif // #ifndef IGNORE_MVVIRTUALDEVICE_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief An interface class to access DMA memory related properties(\b Device specific interface layout only).
/**
 *  This class can be used to configure the way the device works with DMA memory.
 *  This is an advanced feature and shall only be used if necessary or to
 *  fine tune an application to achieve optimal performance.
 *
 *  All frame grabbers belonging to the mvTITAN, mvGAMMA, mvSIGMA and mvDELTA series use a special,
 *  preallocated block of memory(DMA memory) as target for fast image transfer. The total size of
 *  this block has to be set during driver installation. The memory is claimed from the operating
 *  system during startup and is shared between all installed devices.
 *
 *  The memory size needed to achieve optimal performance depends on the image size to grab, the queue depth
 *  and the mode the memory manager is operated in. Valid modes are defined by the enumeration
 *  \b mvIMPACT::acquire::TMemoryManagerMode and can be selected by modifying the property
 *  \b mvIMPACT::acquire::ImageMemoryManager::mode.
 *
 *  \note This class is currently available for frame grabber devices only.
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class ImageMemoryManager : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::ImageMemoryManager object.
    explicit ImageMemoryManager(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : ComponentCollection( pDev ),
        mode(), allocatedDMAMemory_bytes(),
        usedDMAMemory_bytes(), usedBlocks(),
        poolMode(), poolBlockSize_bytes(),
        poolBlockCount()
    {
        DeviceComponentLocator locator( pDev, dltImageMemoryManager );
        m_hRoot = locator.searchbase_id();
        locator.bindComponent( mode, "Mode" );
        locator.bindComponent( allocatedDMAMemory_bytes, "AllocatedDMAMemory_bytes" );
        locator.bindComponent( usedDMAMemory_bytes, "UsedDMAMemory_bytes" );
        locator.bindComponent( usedBlocks, "UsedBlocks" );
        locator.bindComponent( poolMode, "PoolMode" );
        locator.bindComponent( poolBlockSize_bytes, "PoolBlockSize_bytes" );
        locator.bindComponent( poolBlockCount, "PoolBlockCount" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property containing the mode the memory manager is currently operated in.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TMemoryManagerMode.
     */
    PropertyIMemoryManagerMode mode;
    /// \brief An integer property \b (read-only) containing the size of the allocated DMA memory (in bytes).
    PropertyI allocatedDMAMemory_bytes;
    /// \brief An integer property \b (read-only) containing the size of the DMA memory (in bytes) currently used by the device.
    PropertyI usedDMAMemory_bytes;
    /// \brief An integer property \b (read-only) containing the number of individual blocks of DMA memory currently used by the device.
    PropertyI usedBlocks;
    /// \brief An enumerated integer property containing the mode the memory managers memory pool is currently operated in.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TMemoryManagerPoolMode.
     */
    PropertyIMemoryManagerPoolMode poolMode;
    /// \brief An integer property defining the size (in bytes) to use for each block of DMA memory.
    PropertyI poolBlockSize_bytes;
    /// \brief An integer property \b (read-only) containing the number of individual blocks of DMA memory currently available when using the current pool block size and pool mode.
    PropertyI poolBlockCount;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIMemoryManagerMode getMode( void ) const
    {
        return mode;
    }
    PropertyI getAllocatedDMAMemory( void ) const
    {
        return allocatedDMAMemory_bytes;
    }
    PropertyI getUsedDMAMemory( void ) const
    {
        return usedDMAMemory_bytes;
    }
    PropertyI getUsedBlocks( void ) const
    {
        return usedBlocks;
    }
    PropertyIMemoryManagerPoolMode getPoolMode( void ) const
    {
        return poolMode;
    }
    PropertyI getPoolBlockSize( void ) const
    {
        return poolBlockSize_bytes;
    }
    PropertyI getPoolBlockCount( void ) const
    {
        return poolBlockCount;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#   if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)
//-----------------------------------------------------------------------------
/// \brief A more specific class to query information about a \b mvBlueDevice device and its driver(\b Device specific interface layout only).
/**
 *  This class contains a collection of properties providing various information about
 *  a \b mvBlueDevice device and its driver.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class InfoBlueDevice : public InfoBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::InfoBlueDevice object.
    explicit InfoBlueDevice(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : InfoBase( pDev ),
        sensorXRes(), sensorYRes(), sensorColorMode(), sensorType()
    {
        ComponentLocator locator( m_hRoot, "Camera" );
        locator.bindComponent( sensorXRes, "SensorXRes" );
        locator.bindComponent( sensorYRes, "SensorYRes" );
        locator.bindComponent( sensorColorMode, "SensorColorMode" );
        locator.bindComponent( sensorType, "SensorType" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property \b (read-only) containing the horizontal resolution of the camera sensor.
    /**
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     */
    PropertyI sensorXRes;
    /// \brief An integer property \b (read-only) containing the vertical resolution of the camera sensor.
    /**
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     */
    PropertyI sensorYRes;
    /// \brief An enumerated integer property \b (read-only) containing the type of the sensor (color/mono/...)(if known).
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TInfoSensorColorMode.
     *
     *  \note
     *  If nothing is known about this feature, this property will contain \b mvIMPACT::acquire::iscmUnknown.
     *
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     */
    PropertyIInfoSensorColorMode sensorColorMode;
    /// \brief An enumerated integer property \b (read-only) containing the type of sensor chip of the camera (if known).
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TInfoSensorType.
     *
     *  This usually will either be \b mvIMPACT::acquire::istCCD or \b mvIMPACT::acquire::istCMOS.
     *
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     */
    PropertyIInfoSensorType sensorType;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getSensorXRes( void ) const
    {
        return sensorXRes;
    }
    PropertyI getSensorYRes( void ) const
    {
        return sensorYRes;
    }
    PropertyIInfoSensorColorMode getSensorColorMode( void ) const
    {
        return sensorColorMode;
    }
    PropertyIInfoSensorType getSensorType( void ) const
    {
        return sensorType;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)

#   ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A more specific class to query information about a \b mvBlueCOUGAR or \b mvBlueLYNX-M7 device and its driver(\b Device specific interface layout only).
/**
 *  This class contains a collection of properties providing various information about
 *  a \b mvBlueCOUGAR or \b mvBlueLYNX-M7 device and its driver.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class InfoBlueCOUGAR : public InfoBlueDevice
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::InfoBlueCOUGAR object.
    explicit InfoBlueCOUGAR(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : InfoBlueDevice( pDev ), deviceBootloaderVersion(), deviceRFSVersion(),
        deviceKernelVersion(), deviceFPGAVersion(), deviceStatus(), deviceStatusMessage(),
        deviceSensorRevision(), clientConnectionState()
    {
        ComponentLocator locator( m_hRoot, "Camera" );
        locator.bindComponent( deviceBootloaderVersion, "DeviceBootloaderVersion" );
        locator.bindComponent( deviceRFSVersion, "DeviceRFSVersion" );
        locator.bindComponent( deviceKernelVersion, "DeviceKernelVersion" );
        locator.bindComponent( deviceFPGAVersion, "DeviceFPGAVersion" );
        locator.bindComponent( deviceStatus, "DeviceStatus" );
        locator.bindComponent( deviceStatusMessage, "DeviceStatusMessage" );
        locator.bindComponent( deviceSensorRevision, "DeviceSensorRevision" );
        locator.bindComponent( clientConnectionState, "ClientConnectionState" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A 64 bit integer property \b (read-only) containing the current bootloader version of this device.
    /**
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceBootloaderVersion;
    /// \brief A 64 bit integer property \b (read-only) containing the current version of the root file system (RFS) of this device.
    /**
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceRFSVersion;
    /// \brief A 64 bit integer property \b (read-only) containing the current kernel version of this device.
    /**
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceKernelVersion;
    /// \brief A 64 bit integer property \b (read-only) containing the current FPGA version of this device.
    /**
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceFPGAVersion;
    /// \brief A 64 bit integer property \b (read-only) containing the current device status of the sensor head of this device.
    /**
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     *
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceStatus;
    /// \brief A string property \b (read-only) containing the current device status message of the sensor head of this device.
    /**
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     *
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyS deviceStatusMessage;
    /// \brief A 64 bit integer property \b (read-only) containing the current revision of the sensor head of this device.
    /**
     *  If the device has more than one sensor head, and all these sensor heads can be accessed using the current interface,
     *  this property will contain as many values as the devices offers sensor heads.
     *
     *  \note
     *  This feature is only supported by \b mvBlueCOUGAR-P and mvBlueLYNX-M7 devices.
     */
    PropertyI64 deviceSensorRevision;
    /// \brief An integer property \b (read-only) providing information about the current connection state of a remote client.
    /**
     *  A value different from 0 indicates that at least one client is connected to this device. When \b NOT working locally
     *  this property will always contain '1'.
     */
    PropertyI clientConnectionState;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI64 getDeviceBootloaderVersion( void ) const
    {
        return deviceBootloaderVersion;
    }
    PropertyI64 getDeviceRFSVersion( void ) const
    {
        return deviceRFSVersion;
    }
    PropertyI64 getDeviceKernelVersion( void ) const
    {
        return deviceKernelVersion;
    }
    PropertyI64 getDeviceFPGAVersion( void ) const
    {
        return deviceFPGAVersion;
    }
    PropertyI64 getDeviceStatus( void ) const
    {
        return deviceStatus;
    }
    PropertyS getDeviceStatusMessage( void ) const
    {
        return deviceStatusMessage;
    }
    PropertyI64 getDeviceSensorRevision( void ) const
    {
        return deviceSensorRevision;
    }
    PropertyI getClientConnectionState( void ) const
    {
        return clientConnectionState;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A more specific class to query information about a \b mvBlueFOX device and its driver(\b Device specific interface layout only).
/**
 *  This class contains a collection of properties providing various information about
 *  a \b mvBlueFOX device and its driver.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class InfoBlueFOX : public InfoBlueDevice
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::InfoBlueFOX object.
    explicit InfoBlueFOX(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : InfoBlueDevice( pDev ), firmwareVersion(), sensorFPGAVersion(), sensorCaps(),
        deviceSensorRevision(), userEEPROMSize(), deviceTemperature()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( firmwareVersion, "FirmwareVersion" );
        locator.bindSearchBase( locator.searchbase_id(), "Camera" );
        locator.bindComponent( sensorFPGAVersion, "SensorFPGAVersion" );
        locator.bindComponent( sensorCaps, "SensorCaps" );
        locator.bindComponent( deviceSensorRevision, "DeviceSensorRevision" );
        locator.bindComponent( userEEPROMSize, "UserEEPROMSize" );
        locator.bindComponent( deviceTemperature, "DeviceTemperature" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property \b (read-only) containing the firmware version of this device.
    PropertyI firmwareVersion;
    /// \brief An integer property \b (read-only) containing the FPGA version of the camera(if known).
    /**
     *  \note
     *  If nothing is known about this feature, this property will contain '-1'.
     */
    PropertyI sensorFPGAVersion;
    /// \brief An integer property \b (read-only) containing a bit mask containing information about the sensors capabilities(if known).
    /**
     *  \note
     *  If nothing is known about this feature, this property will contain '-1'.
     */
    PropertyIBlueFOXInfoSensorCapabilities sensorCaps;
    /// \brief A 64 bit integer property \b (read-only) containing the current revision of the sensor head of this device.
    /**
     *  \note
     *  This feature is currently not supported by all sensors. If a sensor can't provide this information, the value
     *  of this property will remain 0 at all time.
     */
    PropertyI64 deviceSensorRevision;
    /// \brief An integer property \b (read-only) containing the size of the user EEPROM that can be accessed using the I2C access features of \b mvIMPACT::acquire::I2CControl.
    /**
     *  \note
     *  The size may vary depending on the product. Even different products with the same sensor might report
     *  different values here.
     *
     *  \note
     *  The user EEPROM uses a virtual I2C address of 0x1A2 for write access and 0x1A3 for read access.
     */
    PropertyI userEEPROMSize;
    /// \brief A float property \b (read-only) containing the temperature of the selected temperature sensor on the device in degrees Celsius.
    /**
     *  \note
     *  This property will not be available for every mvBlueFOX. Right now this feature is only implemented for
     *  202d version of the mvBlueFOX.
     */
    PropertyF deviceTemperature;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyI getFirmwareVersion( void ) const
    {
        return firmwareVersion;
    }
    PropertyI getSensorFPGAVersion( void ) const
    {
        return sensorFPGAVersion;
    }
    PropertyIBlueFOXInfoSensorCapabilities getSensorCaps( void ) const
    {
        return sensorCaps;
    }
    PropertyI64 getDeviceSensorRevision( void ) const
    {
        return deviceSensorRevision;
    }
    PropertyI getUserEEPROMSize( void ) const
    {
        return userEEPROMSize;
    }
    PropertyF getDeviceTemperature( void ) const
    {
        return deviceTemperature;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

//-----------------------------------------------------------------------------
/// \brief A class to represent a digital input pin(\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class DigitalInput
//-----------------------------------------------------------------------------
{
    friend class    IOSubSystem;
    HDRV            m_hDrv;
    PropertyI       m_register;
    int             m_bitNumber;
    std::string     m_description;
    explicit        DigitalInput( HDRV hDrv, PropertyI reg, int bitNumber, const std::string desc ) : m_hDrv( hDrv ), m_register( reg ), m_bitNumber( bitNumber ), m_description( desc ) {}
public:
    /// \brief Returns the current state of this input pin.
    /**
     *  \return
     *  - true if the current state of this pin is considered as 'logic 1'
     *  - false otherwise
     */
    bool get( void ) const
    {
        DMR_UpdateDigitalInputs( m_hDrv );
        return m_register.read( m_bitNumber ) != 0;
    }
    /// \brief Returns a description for this digital input.
    /**
     *  This might contain connector descriptions or other information like
     *  e.g. 'Trigger-In(J8.4/J8.5)', which means, this is the trigger input
     *  of connector \a J8 pins \a 4 and \a 5.
     */
    std::string getDescription( void ) const
    {
        return m_description;
    }
};

//-----------------------------------------------------------------------------
/// \brief A class to represent a digital output pin(\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class DigitalOutput
//-----------------------------------------------------------------------------
{
    friend class    IOSubSystem;
    PropertyI       m_register;
    int             m_bitNumber;
    std::string     m_description;
    explicit        DigitalOutput( PropertyI reg, int bitNumber, const std::string desc ) : m_register( reg ), m_bitNumber( bitNumber ), m_description( desc ) {}
public:
    /// \brief Inverts the current state of the digital output and returns the previous state.
    /**
     *  \return
     *  - true if the previous state of this pin was considered as 'logic 1'
     *  - false otherwise
     */
    bool flip( void )
    {
        bool oldState = get();
        m_register.write( !oldState, m_bitNumber );
        return oldState;
    }
    /// \brief Returns the current state of this output pin.
    /**
     *  \return
     *  - true if the current state of this pin is considered as 'logic 1'
     *  - false otherwise
     */
    bool get( void ) const
    {
        return m_register.read( m_bitNumber ) != 0;
    }
    /// \brief Checks if the caller has write/modify access to this digital output.
    /**
     *  Some devices will offer access to digital outputs, but these outputs can't be switched manually.
     *  E.g. a device might offer a digital output that can only be configured to stay either high or
     *  low during frame exposure but can't be switched to high at a certain user defined moment. For
     *  such an output, this function will return false.
     *
     *  If the user calls a function belonging to this class that would modify the state of the
     *  associated digital output, an exception will be raised.
     *
     *  \return
     *  - true if the caller is allowed to call write/modify operation for this component.
     *  - false otherwise.
     */
    bool isWriteable( void ) const
    {
        return m_register.isWriteable();
    }
    /// \brief Sets the output pin to 'logic 1'.
    void set( void )
    {
        m_register.write( 1, m_bitNumber );
    }
    /// \brief Sets the output pin to 'logic 0'.
    void reset( void )
    {
        m_register.write( 0, m_bitNumber );
    }
    /// \brief Returns a description for this digital output.
    /**
     *  This might contain connector descriptions or other information like
     *  e.g. 'CC1(J1)', which means, this is the CameraLink&reg; control
     *  channel 1 of connector \a J1.
     */
    std::string getDescription( void ) const
    {
        return m_description;
    }
};

//-----------------------------------------------------------------------------
/// \brief A class to represent a sync. output pin(\b Device specific interface layout only).
/**
 *  Instances of this class can be used to define and generate sync. signals
 *  at the corresponding outputs of the device.
 *
 *  \note
 *  Instances of this class can't be constructed directly, but must be obtained
 *  via the device specific class derived from \b mvIMPACT::acquire::IOSubSystem.
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class SyncOutput : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class IOSubSystem;
    explicit SyncOutput( HLIST hList ) : ComponentCollection( hList ), frequency_Hz(), lowPart_pc()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( frequency_Hz, "Frequency_Hz" );
        locator.bindComponent( lowPart_pc, "LowPart_pc" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief A float property defining the frequency(in Hertz) for the sync. signal generated by this pin.
    PropertyF frequency_Hz;
    /// \brief A float property defining the width in percent the sync. signal stays low during one period. The output signal generated will be a square pulse.
    PropertyF lowPart_pc;
    PYTHON_ONLY( %mutable; )
    /// \brief Returns a description for this sync. output.
    /**
     *  This might contain connector descriptions or other information like
     *  e.g. 'VD-OUT(J9.24)', which means, this is the VD output at connector J9
     *  pin 24.
     */
    std::string getDescription( void ) const
    {
        return ComponentList( m_hRoot ).name();
    }
#   ifdef DOTNET_ONLY_CODE
    PropertyF getFrequency_Hz( void ) const
    {
        return frequency_Hz;
    }
    PropertyF getLowPart_pc( void ) const
    {
        return lowPart_pc;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to represent one step of a real time control(\b RTCtr) program (\b Device specific interface layout only).
/**
 *  An object of this class represents a single instruction in a program represented
 *  by an \b mvIMPACT::acquire::RTCtrProgram. \b mvIMPACT::acquire::RTCtrProgramStep object can't be constructed
 *  directly, but must be accessed via the function
 *  \b mvIMPACT::acquire::RTCtrProgram::programStep of the program they belong
 *  to.
 *
 *  The instruction this \b mvIMPACT::acquire::RTCtrProgramStep will executed when the program is
 *  executed is defined by the property \b mvIMPACT::acquire::RTCtrProgramStep::opCode. Depending
 *  on the value of this property certain other parameters will influence the
 *  behaviour of this instruction.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class RTCtrProgramStep : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class RTCtrProgram;
    explicit RTCtrProgramStep( HOBJ hObj ) : ComponentCollection( hObj ), address(), frameID(), clocks_us(),
        digitalInputs(), digitalOutputs(), sensorHeads(), opCode(), controllerRegister(),
        registerValue()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( address, "Address" );
        locator.bindComponent( frameID, "FrameID" );
        locator.bindComponent( clocks_us, "Clocks_us" );
        locator.bindComponent( digitalInputs, "DigitalInputs" );
        locator.bindComponent( digitalOutputs, "DigitalOutputs" );
        locator.bindComponent( sensorHeads, "SensorHeads" );
        locator.bindComponent( opCode, "OpCode" );
        locator.bindComponent( controllerRegister, "ControllerRegister" );
        locator.bindComponent( registerValue, "RegisterValue" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property, which defines the absolute jump address within this \b mvIMPACT::acquire::RTCtrProgram.
    /**
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgJumpLoc.
     */
    PropertyI address;
    /// \brief An integer property, which defines the frame ID of the triggered image \b mvIMPACT::acquire::RTCtrProgram.
    /**
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgTriggerSet.
     */
    PropertyI frameID;
    /// \brief An integer property, which defines the waiting time \b mvIMPACT::acquire::RTCtrProgram.
    /**
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgWaitClocks.
     */
    PropertyI clocks_us;
    /// \brief An enumerated integer property to define for which digital input state this program step waits for.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigIOState.
     *
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgWaitDigin.
     */
    PropertyIDigIOState digitalInputs;
    /// \brief An enumerated integer property to define how to set the digital outputs of this device.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigIOState.
     *
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgSetDigout.
     */
    PropertyIDigIOState digitalOutputs;
    /// \brief An enumerated integer property to define which sensor heads to trigger (for devices with more than one sensor head).
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigIOState.
     *
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to \b mvIMPACT::acquire::rtctrlProgTriggerSet
     *  or \b mvIMPACT::acquire::rtctrlProgTriggerReset.
     */
    PropertyIDigIOState sensorHeads;
    /// \brief An enumerated integer property defining the general purpose of this \b mvIMPACT::acquire::RTCtrProgramStep object.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TRTProgOpCodes.
     */
    PropertyIRTProgOpCodes opCode;
    /// \brief An integer property, which defines the index of the controller register to refer in this \b mvIMPACT::acquire::RTCtrProgramStep.
    /**
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSet, \b mvIMPACT::acquire::rtctrlProgRegisterAdd,
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSub, \b mvIMPACT::acquire::rtctrlProgJumpLocOnZero
     *  or \b mvIMPACT::acquire::rtctrlProgJumpLocOnNotZero.
     *
     *  \note
     *  This feature will not be available for every device offering a HRTC. It will be available if the translation
     *  dictionary of the property \b mvIMPACT::acquire::RTCtrProgramStep::opCode contains
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSet, \b mvIMPACT::acquire::rtctrlProgRegisterAdd,
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSub, \b mvIMPACT::acquire::rtctrlProgJumpLocOnZero
     *  or \b mvIMPACT::acquire::rtctrlProgJumpLocOnNotZero.
     *
     *  Read the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyI controllerRegister;
    /// \brief An integer property, which defines the a value to associate with a register access in this \b mvIMPACT::acquire::RTCtrProgramStep.
    /**
     *  This property only is taken into account, if \b mvIMPACT::acquire::RTCtrProgramStep::opCode is set to
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSet, \b mvIMPACT::acquire::rtctrlProgRegisterAdd
     *  or \b mvIMPACT::acquire::rtctrlProgRegisterSub.
     *
     *  \note
     *  This feature will not be available for every device offering a HRTC. It will be available if the translation
     *  dictionary of the property \b mvIMPACT::acquire::RTCtrProgramStep::opCode contains
     *  \b mvIMPACT::acquire::rtctrlProgRegisterSet, \b mvIMPACT::acquire::rtctrlProgRegisterAdd,
     *  or \b mvIMPACT::acquire::rtctrlProgRegisterSub.
     *
     *  Read the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyI registerValue;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyI getAddress( void ) const
    {
        return address;
    }
    PropertyI getFrameID( void ) const
    {
        return frameID;
    }
    PropertyI getClocks_us( void ) const
    {
        return clocks_us;
    }
    PropertyIDigIOState getDigitalInputs( void ) const
    {
        return digitalInputs;
    }
    PropertyIDigIOState getDigitalOutputs( void ) const
    {
        return digitalOutputs;
    }
    PropertyIDigIOState getSensorHeads( void ) const
    {
        return sensorHeads;
    }
    PropertyIRTProgOpCodes getOpCode( void ) const
    {
        return opCode;
    }
    PropertyI getControllerRegister( void ) const
    {
        return controllerRegister;
    }
    PropertyI getRegisterValue( void ) const
    {
        return registerValue;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to represent real time control programs(\b Device specific interface layout only).
/**
 *  Real time control programs can be used to control the way how an when images
 *  are exposed and transmitted to the user.
 *
 *  An \b mvIMPACT::acquire::RTCtrProgram e.g. can be used to achieve a constant frame rate with
 *  a desired frequency. This can be done e.g. by triggering the camera with a constant frequency.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  An appropriate signal for triggering the camera can be generated as follows:
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *
 *  using namespace mvIMPACT::acquire;
 *
 *  void runAt10Hz( Device* pBF )
 *  {
 *    CameraSettingsBlueFOX bfs( pBF );
 *    IOSubSystemBlueFOX bfIOs( pBF );
 *    // define a HRTC program that results in a define image frequency
 *    // the hardware real time controller shall be used to trigger an image
 *    bfs.triggerSource.write( ctsRTCtrl );
 *    // when the hardware real time controller switches the trigger signal to
 *    // high the exposure of the image shall start
 *    bfs.triggerMode.write( ctmOnRisingEdge );
 *
 *    // error checks
 *    if( bfIOs.RTCtrProgramCount() == 0 )
 *    {
 *      // no HRTC controllers available (this never happens for the mvBlueFOX)
 *      return;
 *    }
 *
 *    RTCtrProgram* pRTCtrlProgram = bfIOs.getRTCtrProgram( 0 );
 *    if( !pRTCtrlProgram )
 *    {
 *      // this only should happen if the system is short of memory
 *      return;
 *    }
 *
 *    // start of the program
 *
 *    // we need 5 steps for the program
 *    pRTCtrlProgram->setProgramSize(5);
 *
 *    // wait a certain amount of time to achieve the desired frequency
 *    int progStep = 0;
 *    RTCtrProgramStep* pRTCtrlStep = 0;
 *    pRTCtrlStep = pRTCtrlProgram->programStep( progStep++ );
 *    pRTCtrlStep->opCode.write( rtctrlProgWaitClocks );
 *    pRTCtrlStep->clocks_us.write( 99900 );
 *
 *    // trigger an image
 *    pRTCtrlStep = pRTCtrlProgram->programStep( progStep++ );
 *    pRTCtrlStep->opCode.write( rtctrlProgTriggerSet );
 *
 *    // high time for the trigger signal (should not be smaller than 100 us)
 *    pRTCtrlStep = pRTCtrlProgram->programStep( progStep++ );
 *    pRTCtrlStep->opCode.write( rtctrlProgWaitClocks );
 *    pRTCtrlStep->clocks_us.write( 100 );
 *
 *    // end trigger signal
 *    pRTCtrlStep = pRTCtrlProgram->programStep( progStep++ );
 *    pRTCtrlStep->opCode.write( rtctrlProgTriggerReset );
 *
 *    // restart the program
 *    pRTCtrlStep = pRTCtrlProgram->programStep( progStep++ );
 *    pRTCtrlStep->opCode.write( rtctrlProgJumpLoc );
 *    pRTCtrlStep->address.write( 0 );
 *
 *    // start the program
 *    pRTCtrlProgram->mode.write( rtctrlModeRun );
 *
 *    // Now this camera will deliver images at exactly 10 Hz ( 1 / ( 99900 us + 100us ) )
 *    // when it is constantly feed with image requests.
 *  }
 * \endcode
 *  \endif
 *
 *  Once this program has been defined it can be executed by setting the \a mode
 *  property to \b mvIMPACT::acquire::rtctrlModeRun. As in this example we do not
 *  wait for an external signal the property \b triggerMode
 *  must be set to a value demanding an external signal
 *  ( e.g. \b mvIMPACT::acquire::ctmOnRisingEdge  ) and the  property
 *  \b triggerMode must be set to \b mvIMPACT::acquire::ctsRTCtrl
 *  to inform the driver that the signal generate by the real time program shall
 *  be used as the trigger signal.
 *
 *  \note
 *  \b mvIMPACT::acquire::RTCtrProgram objects can't be created directly! To access an \b mvIMPACT::acquire::RTCtrProgram
 *  associated with a certain device create an instance of an object derived from
 *  \b mvIMPACT::acquire::IOSubSystem and then use the function
 *  \b mvIMPACT::acquire::IOSubSystem::RTCtrProgramCount() to find out if there are
 *  hardware real time controller machines available for this device and if so how
 *  many of them can be accessed. Pointers to the machines then can be obtained by calling the
 *  function \b mvIMPACT::acquire::IOSubSystem::getRTCtrProgram() with the number of the
 *  controller (zero based) to be modified.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class RTCtrProgram : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class IOSubSystem;
    typedef std::vector<RTCtrProgramStep*> RTCtrlProgramStepVector;
#       ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        HDRV                            m_hDrv;
        mutable RTCtrlProgramStepVector m_programSteps;
        PropertyI                       m_programSize;
        HLIST                           m_hRTCtrList;
        HOBJ                            m_programStepList;
        unsigned int                    m_refCnt;
        ReferenceCountedData( HDRV hDrv, HLIST hList ) : m_hDrv( hDrv ), m_programSteps(), m_programSize(),
            m_hRTCtrList( hList ), m_programStepList(), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            RTCtrlProgramStepVector::size_type vSize = m_programSteps.size();
            for( RTCtrlProgramStepVector::size_type i = 0; i < vSize; i++ )
            {
                delete m_programSteps[i];
            }
        }
    }* m_pRefData;
#       endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    void bindPublicProperties( HLIST hList )
    //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( hList );
        locator.bindComponent( mode, "Mode" );
        locator.bindComponent( filename, "Filename" );
        locator.bindComponent( programState, "ProgramState" );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    void updateProgram( void ) const
    //-----------------------------------------------------------------------------
    {
        RTCtrlProgramStepVector::size_type vSize = m_pRefData->m_programSteps.size();
        unsigned int lSize = ComponentList( m_pRefData->m_programStepList ).size();
        if( lSize < vSize )
        {
            // steps have been deleted somewhere...
            for( unsigned int i = lSize; i < vSize; i++ )
            {
                delete m_pRefData->m_programSteps[i];
            }
            m_pRefData->m_programSteps.resize( lSize );
        }
        else if( lSize > vSize )
        {
            // new steps have been created
            ComponentIterator iter( m_pRefData->m_programStepList );
            iter = iter.firstChild();
            // move to the next valid end point
            for( unsigned int i = 0; i < vSize; i++ )
            {
                ++iter;
            }

            while( iter.isValid() )
            {
                m_pRefData->m_programSteps.push_back( new RTCtrProgramStep( iter ) );
                ++iter;
            }
        }
    }
    //-----------------------------------------------------------------------------
    explicit RTCtrProgram( HDRV hDrv, HLIST hList ) : ComponentCollection( hList ),
        m_pRefData( new ReferenceCountedData( hDrv, hList ) ), mode(), filename(), programState()
        //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( hList );
        locator.bindComponent( m_pRefData->m_programSize, "ProgramSize" );
        bindPublicProperties( m_pRefData->m_hRTCtrList );
        m_pRefData->m_programStepList = locator.findComponent( "RTCtrProgram" );
        updateProgram();
    }
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::RTCtrProgram from an exisiting one.
    explicit RTCtrProgram(
        /// [in] A constant reference to the \b mvIMPACT::acquire::RTCtrProgram object, this object shall be created from
        const RTCtrProgram& src ) : ComponentCollection( src.m_hRoot ), m_pRefData( src.m_pRefData ), mode( src.mode ),
        filename( src.filename ), programState( src.programState )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor.
    ~RTCtrProgram()
    {
        dealloc();
    }
#   ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::RTCtrProgram objects
    RTCtrProgram& operator=( const RTCtrProgram& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
            bindPublicProperties( m_pRefData->m_hRTCtrList );
        }
        return *this;
    }
#   endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief A function to define the number of instructions this program should consist of.
    void setProgramSize(
        /// [in] The new number of program instructions
        int newSize )
    {
        m_pRefData->m_programSize.write( newSize );
        updateProgram();
    }
    /// \brief Returns the number of program instructions for this program.
    /**
     *  \return The number of program instructions for this program.
     */
    int getProgramSize( void ) const
    {
        return m_pRefData->m_programSize.read();
    }
    /// \brief Returns a pointer to a program instruction of the program.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A pointer to a program instruction of the program.
     */
    RTCtrProgramStep* programStep(
        /// [in] The index of the program instruction to obtain.
        unsigned int nr ) const
    {
        updateProgram();
        return m_pRefData->m_programSteps.at( nr );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the current state this program is into.
    /**
     *  In order to affect the behaviour of the image acquisition an \b mvIMPACT::acquire::RTCtrProgram must be in
     *  running mode. Apart from that the property \b triggerSource must be set
     *  appropriately.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TRTCtrlModes.
     */
    PropertyIRTCtrlModes mode;
    /// \brief A string property storing the filename for this program.
    /**
     *  \b mvIMPACT::acquire::RTCtrProgram s can be stored and loaded from/to XML files.
     *  To do this, this property must be set to the desired filename.
     *  \sa
     *  \b mvIMPACT::acquire::RTCtrProgram::load, \n \b mvIMPACT::acquire::RTCtrProgram::save
     */
    PropertyS filename;
    /// \brief A string property \b (read-only) containing information about the current state of the program.
    PropertyS programState;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyIRTCtrlModes        getMode( void ) const
    {
        return mode;
    }
    PropertyS                   getFilename( void ) const
    {
        return filename;
    };
    PropertyS                   getProgramState( void ) const
    {
        return programState;
    };
#   endif // #ifdef DOTNET_ONLY_CODE
    /// \brief Loads an existing program specified by the property \b mvIMPACT::acquire::RTCtrProgram::fileName.
    /**
     *  The default file extension for these programs is '*.rtp'. If the user doesn't
     *  specify this file extension, it is appended automatically. Only
     *  files of this type can be loaded by this function.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int load( void )
    {
        return DMR_LoadRTCtrProgram( m_pRefData->m_hDrv, m_pRefData->m_hRTCtrList );
    }
    /// \brief stores the current state of the program under the name specified by the property \b mvIMPACT::acquire::RTCtrProgram::filename
    /**
     *  The default file extension for files stored using this function is '*.rtp'. If
     *  the user doesn't specify this file extension, it is appended automatically. Only
     *  files of this type can be loaded by the function \b mvIMPACT::acquire::RTCtrProgram::load.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int save( void )
    {
        return DMR_SaveRTCtrProgram( m_pRefData->m_hDrv, m_pRefData->m_hRTCtrList );
    }
};

//-----------------------------------------------------------------------------
/// \brief An interface class to model a internal driver event that the user can wait for(\b Device specific interface layout only)(\b deprecated).
/**
 *  \deprecated
 *  This class has been declared \b deprecated and will be removed in future versions
 *  of this interface. A more flexible way of getting informed about changes in driver features
 *  has been added to the interface and should be used instead. An example for this new method can
 *  be found here: \ref Callback.cpp
 *
 *  An event can be anything that occurs at a certain time within the driver or
 *  something that can be reported by the hardware. Typical examples might
 *  be the detection of an external trigger signal, a change at one of the
 *  digital inputs of a device or the start of a VD pulse send from a camera and
 *  detected by a capture device.
 *
 *  Sometimes one or more of these events might be important for an application.
 *  E.g. the user might want to get a notification each time the state of
 *  one of the digital inputs of a device changes in order to read the current
 *  state from the device.
 *
 *  Events will return a timestamp that can be used to synchronize the event
 *  with a certain chronology of other events.
 *
 *  \note
 *  Instances of this class can't be constructed directly. Valid objects must be
 *  obtained via the class \b mvIMPACT::acquire::EventSubSystem.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class Event
//-----------------------------------------------------------------------------
{
    friend class EventSubSystem;
    mutable EventData m_data;
    HDRV m_hDrv;
    TDeviceEventType m_type;
    explicit Event( HDRV hDrv, HOBJ hObjSettings ) : m_data(), m_hDrv( hDrv ), m_type( detNone ), mode(),
        queueDepth(), type()
    {
        memset( &m_data, 0, sizeof( m_data ) );
        ComponentLocator locator( hObjSettings );
        locator.bindComponent( mode, "Mode" );
        locator.bindComponent( queueDepth, "QueueDepth" );
        locator.bindComponent( type, "Type" );
        // buffer this type as it doesn't change and this provides faster access at runtime.
        m_type = type.read();
    }
public:
#   ifndef WRAP_PYTHON
    /// \brief Returns a const reference to the \b mvIMPACT::acquire::EventData structure of this event(<b>deprected</b>).
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::Event.
     *
     *  The referenced \b mvIMPACT::acquire::EventData structure contains additional information about
     *  the image, e.g. a timestamp that defines the time this event has been reported by the device.
     *
     *  \note
     *  Please do \b NOT store this reference in some variable to use it for the
     *  evaluation of the next notification you get for this event as well, as this will \b NOT update
     *  the referenced data. Whenever \b mvIMPACT::acquire::Event::getData is called the function
     *  will make sure that the data in the returned structure is up to date while when working with
     *  an old reference to the \b mvIMPACT::acquire::EventData structure the data in the structure
     *  will refer to the previous result.
     */
    MVIMPACT_DEPRECATED_CPP( const EventData& getData( void ) const );
#   endif // #  ifndef WRAP_PYTHON
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property \b (read-only) defining the mode this event is currently operated in.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::Event.
     *
     *  This property must be used whenever the user wants to receive notification from this event
     *  Valid values for this property are defined by the enumeration
     *  \b mvIMPACT::acquire::TDeviceEventMode.
     */
    PropertyIDeviceEventMode mode;
    /// \brief Currently unsupported.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::Event.
     *
     *  \note
     *  This feature is subject to change! Don't use it!
     */
    PropertyI queueDepth;
    /// \brief An enumerated integer property \b (read-only) containing the type of this event.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::Event.
     *
     *  Valid values for this property are defined by the enumeration
     *  \b mvIMPACT::acquire::TDeviceEventType.
     */
    PropertyIDeviceEventType type;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyIDeviceEventMode getMode( void ) const
    {
        return mode;
    }
    PropertyI getQueueDepth( void ) const
    {
        return queueDepth;
    }
    PropertyIDeviceEventType getType( void ) const
    {
        return type;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class containing the result of a wait call for events (\b Device specific interface layout only)(\b deprecated).
/**
 *  \deprecated
 *  This class has been declared \b deprecated and will be removed in future versions
 *  of this interface. A more flexible way of getting informed about changes in driver features
 *  has been added to the interface and should be used instead. An example for this new method can
 *  be found here: \ref Callback.cpp
 *
 *  Objects of this class will be returned by \b mvIMPACT::acquire::EventSubSystem::waitFor
 *  calls.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class EventWaitResults
//-----------------------------------------------------------------------------
{
    int m_errorCode;
    TDeviceEventType m_waitMask;
    TDeviceEventType m_resultMask;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::EventWaitResults object.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventWaitResults.
     */
    explicit EventWaitResults(
        /// [in] The error code returned by the wait function
        int errorCode,
        /// [in] The wait mask passed to the wait function
        TDeviceEventType waitMask,
        /// [in] The result mask of the wait function call
        TDeviceEventType resultMask );
    /// \brief The error code returned by the wait function.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventWaitResults.
     */
    int errorCode( void ) const
    {
        return m_errorCode;
    }
    /// \brief The wait mask passed to the wait function.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventWaitResults.
     */
    TDeviceEventType waitMask( void ) const
    {
        return m_waitMask;
    }
    /// \brief The result mask of the wait function call.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventWaitResults.
     */
    TDeviceEventType resultMask( void ) const
    {
        return m_resultMask;
    }
};

//-----------------------------------------------------------------------------
/// \brief An interface class that provides access to all event handling related objects and functions(\b Device specific interface layout only)(\b deprecated).
/**
 *  \deprecated
 *  This class has been declared \b deprecated and will be removed in future versions
 *  of this interface. A more flexible way of getting informed about changes in driver features
 *  has been added to the interface and should be used instead. An example for this new method can
 *  be found here: \ref Callback.cpp
 *
 *  Events from the \b mvIMPACT \b Acquire interfaces point of view are certain
 *  things reported by the hardware like e.g. the detection of a frame start or
 *  the change on one of the digital inputs of a device.
 *
 *  This class combines everything that is related \b mvIMPACT \a Acquire s event
 *  handling.
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  //-----------------------------------------------------------------------------
 *  void fn( void )
 *  //-----------------------------------------------------------------------------
 *  {
 *    // create an interface to the event processing
 *    unsigned int frameStartEventCnt = 0;
 *    EventSubSystem ess(getDevicePointerFromSomewhere());
 *    Event* pEvent = ess.getEventByType( detFrameStart );
 *    if( !pEvent )
 *    {
 *      cout << "Frame start events not supported by this device." << endl;
 *      return;
 *    }
 *
 *    pEvent->mode.write( demNotify );
 *    TDeviceEventType type = pEvent->type.read();
 *    TDeviceEventType resultMask;
 *    int timeout_ms = 500;
 *
 *    while( !g_boTerminated )
 *    {
 *      // wait for frame start event
 *      EventWaitResults waitResult = ess.waitFor( timeout_ms, type );
 *      if( ( waitResult.errorCode() == DMR_NO_ERROR ) )
 *      {
 *        // we are just waiting for one event type, thus the result mask ain't interessting
 *        frameStartEventCnt = pEvent->getData().count;
 *        // do something
 *      }
 *      else
 *      {
 *        // no event detected within the given timeout
 *      }
 *    }
 *  }
 * \endcode
 *  \endif
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class EventSubSystem
//-----------------------------------------------------------------------------
{
    typedef std::vector<Event*> EventVector;
    typedef std::map<TDeviceEventType, Event*> EventTypeToEventMap;
    typedef std::map<std::string, Event*> StringToEventMap;
#   ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        EventVector          m_vEvents;
        EventTypeToEventMap  m_mTypeToEvent;
        StringToEventMap     m_mNameToEvent;
        HDRV                 m_hDrv;
        int                  m_refCnt;
        ReferenceCountedData( HDRV hDrv ) : m_vEvents(), m_mTypeToEvent(), m_mNameToEvent(),
            m_hDrv( hDrv ), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            m_mTypeToEvent.clear();
            m_mNameToEvent.clear();
            EventVector::size_type vSize = m_vEvents.size();
            for( EventVector::size_type i = 0; i < vSize; i++ )
            {
                delete m_vEvents[i];
            }
        }
    }* m_pRefData;
#   endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
public:
#   ifndef WRAP_PYTHON
    /// \brief Constructs a new \b mvIMPACT::acquire::EventSubSystem object(\b deprecated).
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     */
    explicit MVIMPACT_DEPRECATED_CPP( EventSubSystem(
                                          /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
                                          Device* pDev ) );
#   endif // #ifndef WRAP_PYTHON
    /// \brief Constructs a new \b mvIMPACT::acquire::EventSubSystem from an existing one.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     *
     *  Internal data is reference counted, thus modifying an event obtained from \a src will also
     *  modify the same event obtained from this newly constructed class.
     */
    explicit EventSubSystem(
        /// [in] A reference to an existing \b mvIMPACT::acquire::EventSubSystem.
        const EventSubSystem& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \deprecated
    ~EventSubSystem()
    {
        dealloc();
    }
#   ifndef WRAP_PYTHON
    /// \brief Allows assignment of one \b mvIMPACT::acquire::EventSubSystem to another.
    /**
     *  \deprecated
     *  Internal data is reference counted, thus modifying an event obtained from \a src will also
     *  modify the same event obtained from this newly constructed class.
     */
    EventSubSystem& operator=(
        /// [in] A reference to an existing \b mvIMPACT::acquire::EventSubSystem.
        const EventSubSystem& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            ++( m_pRefData->m_refCnt );
        }
        return *this;
    }
#   endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Returns a reference to an \b mvIMPACT::acquire::Event based on the index in the internal array of event references.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of recognized events \n
     *  a STL out_of_range exception is thrown.
     *  \endif
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::Event object specifying the event
     *  the given index in the internal list if available.
     *  - an invalid pointer or reference otherwise.
     */
    Event* getEventByIndex(
        /// [in] The index of the event object the user wants to access.
        int index ) const
    {
        return m_pRefData->m_vEvents.at( index );
    }
    /// \brief Returns a reference to an \b mvIMPACT::acquire::Event based on the type of the event.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     *
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::Event object specifying the event of
     *  the given type in the internal list if available.
     *  - an invalid pointer or reference otherwise.
     */
    Event* getEventByType(
        /// [in] The type of the event object the user wants to access.
        TDeviceEventType type ) const
    {
        EventTypeToEventMap::iterator it = m_pRefData->m_mTypeToEvent.find( type );
        if( it != m_pRefData->m_mTypeToEvent.end() )
        {
            return it->second;
        }
        return 0;
    }
    /// \brief Returns a reference to an \b mvIMPACT::acquire::Event based on the type (represented as a string) of the event.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     *
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::Event object specifying the event of
     *  the given type in the internal list if available.
     *  - an invalid pointer or reference otherwise.
     */
    Event* getEventByName(
        /// [in] The name/string representation of the event object the user wants to access.
        const std::string& name ) const
    {
        StringToEventMap::iterator it = m_pRefData->m_mNameToEvent.find( name );
        if( it != m_pRefData->m_mNameToEvent.end() )
        {
            return it->second;
        }
        return 0;
    }
    /// \brief Returns the number of different event types available for the device that has been passed to the constructor of this object.
    /**
     *  \deprecated
     *  All functions belonging to this class have been declared \b deprecated. See detailed description of
     *  \b mvIMPACT::acquire::EventSubSystem.
     */
    unsigned int getEventCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_vEvents.size() );
    }
#   ifndef WRAP_PYTHON
    /// \brief Allows to wait for the occurrence of one or more events(\b deprecated).
    /**
     *  \deprecated
     *  This function has been declared \b deprecated and will be removed in future versions
     *  of this interface. A more flexible way of getting informed about changes in driver features
     *  has been added to the interface and should be used instead. An example for this new method can
     *  be found here: \ref Callback.cpp
     *
     *  This function must be called to wait for one or more events to becomes signalled by the device
     *  driver.
     *
     *  \return A \b mvIMPACT::acquire::EventWaitResults object containing information about the
     *  result of this operation.
     */
    MVIMPACT_DEPRECATED_CPP( EventWaitResults waitFor(
                                 /// [in] The timeout in ms defining the maximum time to be spend waiting
                                 int timeout_ms,
                                 /// [in] A mask specifying the event types to wait for. Here values defined by
                                 /// \b mvIMPACT::acquire::TDeviceEventType can be ORed together.
                                 TDeviceEventType mask ) );
#   endif // #ifndef WRAP_PYTHON
};

#   ifndef DOXYGEN_SHOULD_SKIP_THIS
#       if !defined(WRAP_PYTHON) && !defined(WRAP_DOTNET) // To date, no customer uses Python, so we don't need backward compatibility here.
//-----------------------------------------------------------------------------
inline const EventData& Event::getData( void ) const
//-----------------------------------------------------------------------------
{
    TDMR_ERROR result = DMR_NO_ERROR;
    if( ( result = DMR_EventGetData( m_hDrv, m_type, 0, 0, &m_data, sizeof( m_data ) ) ) != DMR_NO_ERROR )
    {
        std::ostringstream oss;
        oss << "Couldn't obtain event data for event type " << m_type << "(string representation: " << type.name() << ")";
        ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, oss.str() );
    }
    return m_data;
}

//-----------------------------------------------------------------------------
inline EventWaitResults::EventWaitResults( int errorCode, TDeviceEventType waitMask, TDeviceEventType resultMask ) : m_errorCode( errorCode ), m_waitMask( waitMask ), m_resultMask( resultMask ) {}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
inline EventSubSystem::EventSubSystem( Device* pDev ) : m_pRefData( 0 )
//-----------------------------------------------------------------------------
{
    if( !pDev->isOpen() )
    {
        pDev->open();
    }

    m_pRefData = new ReferenceCountedData( pDev->hDrv() );

    TDMR_ERROR result;
    HLIST hList;
    if( ( result = DMR_FindList( pDev->hDrv(), 0, dmltEventSubSystemSettings, 0, &hList ) ) == DMR_NO_ERROR )
    {
        ComponentIterator it( hList );
        it = it.firstChild();
        while( it.isValid() )
        {
            Event* p = new Event( pDev->hDrv(), it.hObj() );
            m_pRefData->m_mTypeToEvent.insert( std::make_pair( p->type.read(), p ) );
            m_pRefData->m_mNameToEvent.insert( std::make_pair( p->type.readS(), p ) );
            m_pRefData->m_vEvents.push_back( p );
            ++it;
        }
    }
}

//-----------------------------------------------------------------------------
inline EventWaitResults EventSubSystem::waitFor( int timeout_ms, TDeviceEventType mask )
//-----------------------------------------------------------------------------
{
    TDeviceEventType resultMask;
    int errorCode = DMR_EventWaitFor( m_pRefData->m_hDrv, timeout_ms, mask, 0, 0, &resultMask );
    return EventWaitResults( errorCode, mask, resultMask );
}
#       endif // #if !defined(WRAP_PYTHON) && !defined(WRAP_DOTNET)
#   endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS

//-----------------------------------------------------------------------------
/// \brief A class to configure the creation of digital signals passed to one or more of the digital outputs of a device(\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class PulseStartConfiguration : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class IOSubSystem;
    explicit PulseStartConfiguration( HLIST hList, int nr ) : ComponentCollection( hList ), m_nr( nr ),
        pulseStartTrigger(), digitalSignal(), divider(), frequency_Hz(), triggerMoment()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( pulseStartTrigger, "PulseStartTrigger" );
        locator.bindComponent( digitalSignal, "DigitalSignal" );
        locator.bindComponent( divider, "Divider" );
        locator.bindComponent( frequency_Hz, "Frequency_Hz" );
        locator.bindComponent( triggerMoment, "TriggerMoment" );
    }
    int m_nr;
public:
    /// \brief Returns the internal number associated with this pulse start configuration
    int number( void ) const
    {
        return m_nr;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property that will define how/when a digital signal is generated by the device.
    /**
     *  When certain digital output signals
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TPulseStartTrigger.
     *
     *  \note
     *  Not every device will offer the same options.
     *  Check for valid modes by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIPulseStartTrigger::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyIPulseStartTrigger pulseStartTrigger;
    /// \brief An enumerated property to define one or more signals that will trigger the creation of the associated output signals.
    /**
     *  This property will be visible and thus active when \b mvIMPACT::acquire::PulseStartConfiguration::digitalSignal is switched
     *  to \b mvIMPACT::acquire::pstDigitalSignal.
     *
     *  Depending on the device one or more signals can be defined as 'active for output signal creation'. When more than one signal can be
     *  defined, calling \b mvIMPACT::acquire::PropertyIDigitalSignal::allowsValueCombinations will return true.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigitalSignal.
     *
     *  \b mvHYPERION \b specific:
     *  Currently when more than one signal can be defined here, these signals will have an 'or relationship', which means that when any of these
     *  signals is detected, it will trigger the creation of the associated output signals.
     *
     *  The following signals are defined:
     *  <table width="100%">
     *  <tr>
     *    <th>enum Value</th>
     *    <th>Associated signal</th>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal1</td>
     *    <td>Line Valid(J1)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal2</td>
     *    <td>Line Valid(J2)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal3</td>
     *    <td>Frame Valid(J1)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal4</td>
     *    <td>Frame Valid(J2)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal5</td>
     *    <td>Trigger-In(J3.4/J3.5)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal6</td>
     *    <td>Sync-In(J3.6/J3.7)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal7</td>
     *    <td>Trigger-In(J4.4/J4.5)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal8</td>
     *    <td>Sync-In(J4.6/J4.7)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal9</td>
     *    <td>DigIn0(J6.9)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal10</td>
     *    <td>DigIn1(J6.10)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal11</td>
     *    <td>DigIn2(J6.11)</td>
     *  </tr>
     *  <tr>
     *    <td>mvIMPACT::acquire::dsSignal12</td>
     *    <td>DigIn3(J6.12)</td>
     *  </tr>
     *  </table>
     */
    PropertyIDigitalSignal digitalSignal;
    /// \brief An integer property defining a factor by which the external input signal shall divided.
    /**
     *  Every \a divider-value pulse will be used then. All others will be skipped.
     */
    PropertyI divider;
    /// \brief A float property defining the frequency in Hz. for the creation of the output signals associated with configuration.
    /**
     *  This property will be visible and thus active when \b mvIMPACT::acquire::PulseStartConfiguration::digitalSignal is switched
     *  to \b mvIMPACT::acquire::pstPeriodically.
     */
    PropertyF frequency_Hz;
    /// \brief An enumerated integer property defining the exact moment when this pulse start configuration shall be triggered.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TTriggerMoment.
     */
    PropertyITriggerMoment triggerMoment;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyIPulseStartTrigger getPulseStartTrigger( void ) const
    {
        return pulseStartTrigger;
    }
    PropertyIDigitalSignal getDigitalSignal( void ) const
    {
        return digitalSignal;
    }
    PropertyI getDivider( void ) const
    {
        return divider;
    }
    PropertyF getFrequency_Hz( void ) const
    {
        return frequency_Hz;
    }
    PropertyITriggerMoment getTriggerMoment( void ) const
    {
        return triggerMoment;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class to handle digital inputs and outputs(\b Device specific interface layout only).
/**
 *  This class can't be instantiated by the user, but acts as a base class for
 *  hardware specific classes. Please use a device specific class suitable for the device
 *  you are working with instead. The documentation of the device specific class will also
 *  contain some example code.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class IOSubSystem
//-----------------------------------------------------------------------------
{
protected:
#   ifndef DOXYGEN_SHOULD_SKIP_THIS
    typedef std::vector<RTCtrProgram*> RTCtrProgramVector;
    typedef std::vector<DigitalInput*> DigitalInputVector;
    typedef std::vector<DigitalOutput*> DigitalOutputVector;
    typedef std::vector<SyncOutput*> SyncOutputVector;
    typedef std::vector<PulseStartConfiguration*> PulseStartConfigurationVector;
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        HDRV                          m_hDrv;
        DigitalInputVector            m_vInputs;
        DigitalOutputVector           m_vOutputs;
        SyncOutputVector              m_vHDOutputs;
        SyncOutputVector              m_vVDOutputs;
        mutable RTCtrProgramVector    m_RTCtrPrograms;
        PulseStartConfigurationVector m_PulseStartConfigurations;
        unsigned int                  m_refCnt;
        ReferenceCountedData( HDRV hDrv ) : m_hDrv( hDrv ), m_vInputs(), m_vOutputs(),
            m_vHDOutputs(), m_vVDOutputs(), m_RTCtrPrograms(), m_PulseStartConfigurations(), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            DigitalInputVector::size_type vInputsSize = m_vInputs.size();
            for( DigitalInputVector::size_type i = 0; i < vInputsSize; i++ )
            {
                delete m_vInputs[i];
            }

            DigitalOutputVector::size_type vOutputsSize = m_vOutputs.size();
            for( DigitalOutputVector::size_type j = 0; j < vOutputsSize; j++ )
            {
                delete m_vOutputs[j];
            }

            SyncOutputVector::size_type vSyncOutputsSize = m_vHDOutputs.size();
            for( SyncOutputVector::size_type k = 0; k < vSyncOutputsSize; k++ )
            {
                delete m_vHDOutputs[k];
            }

            vSyncOutputsSize = m_vVDOutputs.size();
            for( SyncOutputVector::size_type l = 0; l < vSyncOutputsSize; l++ )
            {
                delete m_vVDOutputs[l];
            }

            RTCtrProgramVector::size_type vRTCProgSize = m_RTCtrPrograms.size();
            for( RTCtrProgramVector::size_type m = 0; m < vRTCProgSize; m++ )
            {
                delete m_RTCtrPrograms[m];
            }

            PulseStartConfigurationVector::size_type vPulseStartConfigurationsSize = m_PulseStartConfigurations.size();
            for( PulseStartConfigurationVector::size_type n = 0; n < vPulseStartConfigurationsSize; n++ )
            {
                delete m_PulseStartConfigurations[n];
            }
        }
    }* m_pRefData;
    //-----------------------------------------------------------------------------
    explicit IOSubSystem( Device* pDev ) : m_pRefData( 0 )
        //-----------------------------------------------------------------------------
    {
        if( !pDev->isOpen() )
        {
            pDev->open();
        }
        m_pRefData = new ReferenceCountedData( pDev->hDrv() );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    void registerAllRTCtrPrograms( HDRV hDrv, HLIST hList )
    {
        if( hList != INVALID_ID )
        {
            unsigned int cnt = ComponentList( hList ).size();
            for( unsigned int k = 0; k < cnt; k++ )
            {
                if( DMR_FindList( hDrv, 0, dmltRTCtr, k, &hList ) == DMR_NO_ERROR )
                {
                    registerRTCtrProgram( hDrv, hList );
                }
            }
        }
    }
    void registerDigitalInput( PropertyI reg, int nr, const std::string& desc )
    {
        m_pRefData->m_vInputs.push_back( new DigitalInput( m_pRefData->m_hDrv, reg, nr, desc ) );
    }
    void registerDigitalOutput( PropertyI reg, int nr, const std::string& desc )
    {
        m_pRefData->m_vOutputs.push_back( new DigitalOutput( reg, nr, desc ) );
    }
    void registerHDOutput( HLIST hList )
    {
        m_pRefData->m_vHDOutputs.push_back( new SyncOutput( hList ) );
    }
    void registerRTCtrProgram( HDRV hDrv, HLIST hList )
    {
        m_pRefData->m_RTCtrPrograms.push_back( new RTCtrProgram( hDrv, hList ) );
    }
    void registerPulseStartConfiguration( HLIST hList )
    {
        m_pRefData->m_PulseStartConfigurations.push_back( new PulseStartConfiguration( hList, static_cast<int>( m_pRefData->m_PulseStartConfigurations.size() ) ) );
    }
    void registerVDOutput( HLIST hList )
    {
        m_pRefData->m_vVDOutputs.push_back( new SyncOutput( hList ) );
    }
#   endif // DOXYGEN_SHOULD_SKIP_THIS
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::IOSubSystem from and exisiting one.
    explicit IOSubSystem(
        /// [in] A constant reference to the \b mvIMPACT::acquire::IOSubSystem object, this object shall be created from
        const IOSubSystem& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    virtual ~IOSubSystem()
    {
        dealloc();
    }
#   ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::IOSubSystem objects
    IOSubSystem& operator=( const IOSubSystem& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
        }
        return *this;
    }
#   endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::PulseStartConfiguration associated with this device.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     */
    PulseStartConfiguration* getPulseStartConfiguration(
        /// [in] The number of the pulse start configuration to return
        unsigned int nr ) const
    {
        return m_pRefData->m_PulseStartConfigurations.at( nr );
    }
    /// \brief Returns the number of \b mvIMPACT::acquire::PulseStartConfiguration objects available for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int getPulseStartConfigurationCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_PulseStartConfigurations.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::RTCtrProgram associated with this device.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     */
    RTCtrProgram* getRTCtrProgram(
        /// [in] The number of the real time controller program to return
        unsigned int nr ) const
    {
        return m_pRefData->m_RTCtrPrograms.at( nr );
    }
    /// \brief Returns the number of \b mvIMPACT::acquire::RTCtrProgram s available for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int RTCtrProgramCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_RTCtrPrograms.size() );
    }
    /// \brief Returns the number of \b mvIMPACT::acquire::DigitalInput s available for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int getInputCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_vInputs.size() );
    }
    /// \brief Returns the current state of the digital input register.
    /**
     *  This function can be used to read all digital inputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital inputs 0 and 1 are
     *  currently have a voltage considered as logical '1' applied to them.
     */
    virtual unsigned int readInputRegister( void ) const = 0;
    /// \brief Returns a const pointer to a \b mvIMPACT::acquire::DigitalInput object.
    /**
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \note
     *  The possibility to access the digital inputs of a device as single objects is
     *  just provided for convenience. The same thing can be achieved by calling the
     *  function \b mvIMPACT::acquire::IOSubSystem::readInputRegister.
     */
    const DigitalInput* input(
        /// [in] The number of the digital input.
        unsigned int nr ) const
    {
        return m_pRefData->m_vInputs.at( nr );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::DigitalOutput object.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \note
     *  The possibility to access the digital outputs of a device as single objects is
     *  just provided for convenience. The same thing can be achieved by calling the function
     *  \b mvIMPACT::acquire::IOSubSystem::writeOutputRegister with the corresponding bit mask.
     */
    DigitalOutput* output(
        /// [in] The number of the digital output.
        unsigned int nr ) const
    {
        return m_pRefData->m_vOutputs.at( nr );
    }
    /// \brief Returns the number of digital outputs available for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int getOutputCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_vOutputs.size() );
    }
    /// \brief Returns the current state of the digital output register.
    /**
     *  This function can be used to read all digital outputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital output 0 and 1 are
     *  currently set to high.
     */
    virtual unsigned int readOutputRegister( void ) const = 0;
    /// \brief Alters the state of the digital output register.
    /**
     *  This function can be used to alter the state of certain (or all) digital outputs
     *  with a single function call. By working with the parameter \a mask certain bits
     *  can remain unaffected.
     *
     *  \b EXAMPLE
     *
     *  If the previous state of a digital output register was '5', which means that outputs
     *  0 and 2 are set to high a call to
     *  \b mvIMPACT::acquire::IOSubSystem::writeOutputRegister with
     *  \a value set to 2 and \a mask set to 3 would result in '6' as the new value for the
     *  output register. Pin 2 will remain unaffected, as \a mask states just to modify
     *  pin 0 and 1 ( 3 = 00000011 binary ).
     *
     * \code
     *  currentValue = 5, mask = 3, value = 2
     *  ( currentValue & ~mask ) | value -> new Value
     *  ( 00000101b(current value) & 11111100(~mask) ) | 00000010(value) -> 0000110(new value of the register)
     * \endcode
     */
    virtual void writeOutputRegister(
        /// [in] The value to be applied to the output register
        unsigned int value,
        /// [in] The mask to specify which pins to modify. A bit
        /// set to '1' in this parameter means that the
        /// state of this output has to adopt its style according to
        /// the value in \a value.
        unsigned int mask = UINT_MAX ) = 0;
};

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class to handle the digital inputs and outputs for \b mvBlueFOX USB cameras(\b Device specific interface layout only).
/**
 *  \if DOXYGEN_CPP_DOCUMENTATION
 *  A sample to show how to work with this class and the mvBlueFOX (Creating and instance
 *  of this class for another device might aise an exception):
 *
 * \code
 *  #include <iostream>
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *
 *  using namespace std;
 *  using namespace mvIMPACT::acquire;
 *
 *  void doSomeIOStuff( Device* pBF )
 *  {
 *    IOSubSystemBlueFOX bfIOs( pBF );
 *    cout << "output 0 was " << bfIOs.output(0)->get() << " and is now(after flipping) ";
 *    bfIOs.output(0)->flip();
 *    cout << bfIOs.output(0)->get() << endl;
 *    cout << "input threshold(" << bfIOs.digitalInputThreshold.readS() << ") set to " << bfdit10V << endl;
 *    if( bfIOs.digitalInputThreshold.isWriteable() ) // mvBlueFOX-M doesn't support modifying the input threshold
 *    {
 *      bfIOs.digitalInputThreshold.write( bfdit10V );
 *      cout << "input threshold is now " << bfIOs.digitalInputThreshold.readS() << endl;
 *    }
 *    cout << "input 0 is " << bfIOs.input(0)->get() << endl;
 *    cout << "press ENTER." << endl;
 *    cin.get();
 *    cout << "input 0 is " << bfIOs.input(0)->get() << endl;
 *    cout << "press ENTER." << endl;
 *    cin.get();
 *    cout << "input 0 is " << bfIOs.input(0)->get() << endl;
 *    // switch on digital out 0 and 1 ( 3 = 00000011 binary )
 *    bfIOs.writeOutputRegister( 3 );
 *    cout << "outputs: " << bfIOs.readOutputRegister() << ", inputs: " << bfIOs.readInputRegister();
 *    // the next line will reset bit 0 only, resulting in output 1 to remain high
 *    // as the mask parameter of '1' defines that only the LSB of the register will be
 *    // affected by the first parameter of the function call
 *    bfIOs.writeOutputRegister( 0, 1 );
 *    cout << "outputs: " << bfIOs.readOutputRegister() << ", inputs " << bfIOs.readInputRegister() << endl;
 *  }
 * \endcode
 *  \endif
 *
 *  \note
 *  The \b mvBlueFOX defines an additional property \b mvIMPACT::acquire::CameraSettingsBlueFOX::flashMode.
 *  This might affect the state of the digital outputs as well.
 *
 *  If a digital output is defined to be 'logic 0' but the flash mode is switched on
 *  for the same digital output, the output will be high during the expose time of
 *  the camera sensor. If a digital output is defined to be 'logic 1' the output will
 *  be high all the time.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class IOSubSystemBlueFOX : public IOSubSystem
//-----------------------------------------------------------------------------
{
private:
    unsigned int readRegister( const PropertyI& reg ) const
    {
        std::vector<int> v;
        reg.read( v );
        std::vector<int>::size_type vSize = v.size();
        unsigned int result = 0;
        for( std::vector<int>::size_type i = 0; i < vSize; i++ )
        {
            result |= ( ( v[i] & 1 ) << i );
        }
        return result;
    }
    unsigned int readRegisterAtomic( const PropertyI& reg ) const
    {
        std::vector<int> v;
        reg.read( v, true );
        std::vector<int>::size_type vSize = v.size();
        unsigned int result = 0;
        for( std::vector<int>::size_type i = 0; i < vSize; i++ )
        {
            result |= ( ( v[i] & 1 ) << i );
        }
        return result;
    }
    PropertyI m_inputRegister;
    PropertyI m_outputRegister;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::IOSubSystemBlueFOX object.
    explicit IOSubSystemBlueFOX(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : IOSubSystem( pDev ), m_inputRegister(), m_outputRegister(), digitalInputThreshold()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        locator.bindComponent( digitalInputThreshold, "DigitalInputThreshold" );
        locator.bindComponent( m_inputRegister, "DigitalInputs" );
        locator.bindComponent( m_outputRegister, "DigitalOutputs" );

        // register all input pins
        unsigned int pinCount = m_inputRegister.valCount();
        for( unsigned int i = 0; i < pinCount; i++ )
        {
            std::ostringstream oss;
            oss << "DigitialInput" << i;
            registerDigitalInput( m_inputRegister, i, oss.str() );
        }

        // register all output pins
        pinCount = m_outputRegister.valCount();
        for( unsigned int j = 0; j < pinCount; j++ )
        {
            std::ostringstream oss;
            oss << "DigitalOutput" << j;
            registerDigitalOutput( m_outputRegister, j, oss.str() );
        }

        registerAllRTCtrPrograms( pDev->hDrv(), locator.findComponent( "HardwareRealTimeController" ) );
    }
    /// \brief Returns the current state of the digital input register.
    /**
     *  This function can be used to read all digital inputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital inputs 0 and 1 are
     *  currently have a voltage considered as logical '1' applied to them.
     */
    virtual unsigned int readInputRegister( void ) const
    {
        DMR_UpdateDigitalInputs( m_pRefData->m_hDrv );
        return readRegister( m_inputRegister );
    }
    /// \brief Returns the current state of the digital input register.
    /**
     *  This function can be used to read all digital inputs as a single value \b at the same time. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital inputs 0 and 1 are
     *  currently have a voltage considered as logical '1' applied to them.
     */
    unsigned int readInputRegisterAtomic( void ) const
    {
        DMR_UpdateDigitalInputs( m_pRefData->m_hDrv );
        return readRegisterAtomic( m_inputRegister );
    }
    /// \brief Returns the current state of the digital output register.
    /**
     *  This function can be used to read all digital outputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital output 0 and 1 are
     *  currently set to high.
     */
    virtual unsigned int readOutputRegister( void ) const
    {
        return readRegister( m_outputRegister );
    }
    /// \brief Returns the current state of the digital output register.
    /**
     *  This function can be used to read all digital outputs as a single value \b at the same time. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital output 0 and 1 are
     *  currently set to high.
     */
    unsigned int readOutputRegisterAtomic( void ) const
    {
        return readRegisterAtomic( m_outputRegister );
    }
    /// \brief Alters the state of the digital output register.
    /**
     *  This function can be used to alter the state of certain (or all) digital outputs
     *  with a single function call. By working with the parameter \a mask certain bits
     *  can remain unaffected.
     *
     *  \b EXAMPLE
     *
     *  If the previous state of a digital output register was '5', which means that outputs
     *  0 and 2 are set to high a call to
     *  \b mvIMPACT::acquire::IOSubSystem::writeOutputRegister with
     *  \a value set to 2 and \a mask set to 3 would result in '6' as the new value for the
     *  output register. Pin 2 will remain unaffected, as \a mask states just to modify
     *  pin 0 and 1 ( 3 = 00000011 binary ).
     */
    void writeOutputRegister(
        /// [in] The value to be applied to the output register
        unsigned int value,
        /// [in] The mask to specify which pins to modify. A bit
        /// set to '1' in this parameter means that the
        /// state of this output has to adopt its style according to
        /// the value in \a value.
        unsigned int mask = UINT_MAX )
    {
        unsigned int newVal = ( readRegister( m_outputRegister ) & ~mask ) | ( mask & value );
        DigitalOutputVector::size_type vSize = m_pRefData->m_vOutputs.size();
        std::vector<int> v( vSize );
        for( std::vector<int>::size_type i = 0; i < vSize; i++ )
        {
            v[i] = ( ( newVal & ( 1 << i ) ) != 0 );
        }
        m_outputRegister.write( v );
    }
    /// \brief Alters the state of the digital output register.
    /**
     *  This function can be used to alter the state of certain (or all) digital outputs
     *  with a single function call \b at the same time. By working with the parameter \a mask
     *  certain bits can remain unaffected.
     *
     *  \b EXAMPLE
     *
     *  If the previous state of a digital output register was '5', which means that outputs
     *  0 and 2 are set to high a call to
     *  \b mvIMPACT::acquire::IOSubSystem::writeOutputRegister with
     *  \a value set to 2 and \a mask set to 3 would result in '6' as the new value for the
     *  output register. Pin 2 will remain unaffected, as \a mask states just to modify
     *  pin 0 and 1 ( 3 = 00000011 binary ).
     */
    void writeOutputRegisterAtomic(
        /// [in] The value to be applied to the output register
        unsigned int value,
        /// [in] The mask to specify which pins to modify. A bit
        /// set to '1' in this parameter means that the
        /// state of this output has to adopt its style according to
        /// the value in \a value.
        unsigned int mask = UINT_MAX )
    {
        unsigned int newVal = ( readRegister( m_outputRegister ) & ~mask ) | ( mask & value );
        DigitalOutputVector::size_type vSize = m_pRefData->m_vOutputs.size();
        std::vector<int> v( vSize );
        for( std::vector<int>::size_type i = 0; i < vSize; i++ )
        {
            v[i] = ( ( newVal & ( 1 << i ) ) != 0 );
        }
        m_outputRegister.write( v, true );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the threshold for the digital inputs in Volt.
    /**
     *  If a voltage applied to the digital input lies above the threshold this pin
     *  will be considered as 'logic 1' otherwise it will be considered as 'logic 0'.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBlueFOXDigitalInputThreshold.
     *
     *  \note
     *  This property is \b read-only for \b mvBlueFOX-M devices and will always be 2V.
     */
    PropertyIBlueFOXDigitalInputThreshold digitalInputThreshold;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIBlueFOXDigitalInputThreshold getDigitalInputThreshold( void ) const
    {
        return digitalInputThreshold;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #  ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#   if !defined(IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)
//-----------------------------------------------------------------------------
/// \brief A base class to handle the digital inputs and outputs for devices (\b Device specific interface layout only).
/**
 *  This class provides a convenient basic access to the digital inputs and
 *  outputs of almost every device supported by this interface.
 *
 *  \note
 *  This class may \b NOT be used for mvBlueFOX devices.
 *
 *  \note
 *  The number of digital inputs and outputs available will vary significantly
 *  from device to device. Some devices might not even have digital inputs and/or
 *  digital outputs. Therefore it's crucial to check for available pins before
 *  using them. The corresponding functions to do that can be found in the
 *  base class \b mvIMPACT::acquire::IOSubSystem.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *  #include <iostream>
 *
 *  using namespace std;
 *  using namespace mvIMPACT::acquire;
 *
 *  // lists all available digital inputs and outputs and their description
 *  void listIOPins( Device* pDev )
 *  {
 *    IOSubSystemCommon io(pDev);
 *    unsigned int iCnt = io.getInputCount();
 *    cout << "available digital inputs: " << iCnt << endl;
 *    for( unsigned int d=0; d<iCnt; d++ )
 *    {
 *      out << io.input(d)->getDescription() << endl;
 *    }
 *    unsigned int oCnt = io.getOutputCount();
 *    cout << "available digital outputs: " << oCnt << endl;
 *    for( unsigned int e=0; e<oCnt; e++ )
 *    {
 *      DigitalOutput* pOutput = io.output(e);
 *      out << "setting " << pOutput->getDescription() << " to high" << endl;
 *      // set this output to logic '1'
 *      pOutput->set();
 *    }
 *  }
 * \endcode
 *  \endif
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class IOSubSystemCommon : public IOSubSystem
//-----------------------------------------------------------------------------
{
    ComponentList m_inputs;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::IOSubSystemCommon object.
    explicit IOSubSystemCommon(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : IOSubSystem( pDev ), m_inputs()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        HOBJ hObj = locator.findComponent( "PulseStartConfigurations" );
        if( hObj != INVALID_ID )
        {
            ComponentIterator itPulseConfigs( hObj );
            itPulseConfigs = itPulseConfigs.firstChild();
            while( itPulseConfigs.isValid() )
            {
                registerPulseStartConfiguration( itPulseConfigs.hObj() );
                ++itPulseConfigs;
            }
        }

        locator.bindComponent( m_inputs, "DigitalInputs", 0, 1 );
        if( m_inputs.isValid() )
        {
            ComponentIterator it( m_inputs );
            it = it.firstChild();
            while( it.isValid() )
            {
                registerDigitalInput( PropertyI( it ), 0, it.name() );
                ++it;
            }
        }

        ComponentList outputs;
        locator.bindComponent( outputs, "DigitalOutputs", 0, 1 );
        if( outputs.isValid() )
        {
            ComponentIterator it( outputs );
            it = it.firstChild();
            while( it.isValid() )
            {
                if( it.isList() )
                {
                    ComponentLocator outputStateLocator( it );
                    registerDigitalOutput( PropertyI( outputStateLocator.findComponent( "State" ) ), 0, it.name() );
                }
                ++it;
            }
        }

        registerAllRTCtrPrograms( pDev->hDrv(), locator.findComponent( "HardwareRealTimeController" ) );
    }
    /// \brief Returns the current state of the digital input register.
    /**
     *  This function can be used to read all digital inputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital inputs 0 and 1 are
     *  currently have a voltage considered as logical '1' applied to them.
     *
     *  \return The current state of the digital inputs where bit 0 in the result represents
     *  the state of digital input 0, bit 1 the state of digital input 1 a.s.o.
     */
    virtual unsigned int readInputRegister( void ) const
    {
        DMR_UpdateDigitalInputs( m_pRefData->m_hDrv );
        if( !m_inputs.isValid() )
        {
            return 0;
        }
        unsigned int result = 0;
        ComponentIterator it( m_inputs );
        int i = 0;
        it = it.firstChild();
        while( it.isValid() )
        {
            PropertyI input( it );
            result |= ( ( input.read() & 1 ) << i );
            ++it;
            ++i;
        }
        return result;
    }
    /// \brief Returns the current state of the digital output register.
    /**
     *  This function can be used to read all digital outputs as a single value. Each
     *  bit represents the status of one digital output pin.
     *
     *  \b EXAMPLE
     *
     *  A value of '3' returned by this function means that digital output 0 and 1 are
     *  currently set to high.
     *
     *  \return The current state of the digital outputs where bit 0 in the result represents
     *  the state of digital output 0, bit 1 the state of digital output 1 a.s.o.
     */
    virtual unsigned int readOutputRegister( void ) const
    {
        unsigned int result = 0;
        DigitalOutputVector::size_type vSize = m_pRefData->m_vOutputs.size();
        for( DigitalOutputVector::size_type i = 0; i < vSize; i++ )
        {
            result |= ( ( m_pRefData->m_vOutputs[i]->get() & 1 ) << i );
        }
        return result;
    }
    /// \brief Alters the state of the digital output register.
    /**
     *  This function can be used to alter the state of certain (or all) digital outputs
     *  with a single function call. By working with the parameter \a mask certain bits
     *  can remain unaffected.
     *
     *  \b EXAMPLE
     *
     *  If the previous state of a digital output register was '5', which means that outputs
     *  0 and 2 are set to high a call to
     *  \b mvIMPACT::acquire::IOSubSystem::writeOutputRegister with
     *  \a value set to 2 and \a mask set to 3 would result in '6' as the new value for the
     *  output register. Pin 2 will remain unaffected, as \a mask states just to modify
     *  pin 0 and 1 ( 3 = 00000011 binary ).
     */
    void writeOutputRegister(
        /// [in] The value to be applied to the output register
        unsigned int value,
        /// [in] The mask to specify which pins to modify. A bit
        /// set to '1' in this parameter means that the
        /// state of this output has to adopt its style according to
        /// the value in \a value.
        unsigned int mask = UINT_MAX )
    {
        DigitalOutputVector::size_type vSize = m_pRefData->m_vOutputs.size();
        for( DigitalOutputVector::size_type i = 0; i < vSize; i++ )
        {
            if( mask & ( 1 << i ) )
            {
                if( value & ( 1 << i ) )
                {
                    m_pRefData->m_vOutputs[i]->set();
                }
                else
                {
                    m_pRefData->m_vOutputs[i]->reset();
                }
            }
        }
    }
};
#   endif // #i f !defined(IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)

#   ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION
/// \brief A class to create complex digital output signals(\b Device specific interface layout only).
/**
 *  Instances of this class can be used to create more complex digital output
 *  related signals like a single pulse or sequence of pulses on a certain pin
 *  as a reaction on a signal on one of the digital inputs of a device.
 *
 *  The availability of the features provided by this class heavily depends on the
 *  used capture device. Therefore it's crucial to check if a desired feature is available
 *  at all. This class either provides the functions needed for these checks or its
 *  functions will return appropriate error codes.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class OutputSignalGeneratorBlueDevice
//-----------------------------------------------------------------------------
{
#       ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct OutputProperties
            //-----------------------------------------------------------------------------
    {
        PropertyIDeviceDigitalOutputMode mode;
        PropertyIBoolean inverter;
        OutputProperties( HLIST hOutput ) : mode()
        {
            ComponentLocator locator( hOutput );
            locator.bindComponent( mode, "Mode" );
            locator.bindComponent( inverter, "Inverter" );
        }
    };
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        std::map<std::string, OutputProperties*> m_mOutputs;
        unsigned int                             m_refCnt;
        ReferenceCountedData() : m_mOutputs(), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            std::map<std::string, OutputProperties*>::iterator itEnd = m_mOutputs.end();
            std::map<std::string, OutputProperties*>::iterator it = m_mOutputs.begin();
            while( it != itEnd )
            {
                delete it->second;
                ++it;
            }
        }
    }* m_pRefData;
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    OutputProperties* getOutputProperties( DigitalOutput* pOutput ) const
    //-----------------------------------------------------------------------------
    {
        if( pOutput )
        {
            std::map<std::string, OutputProperties*>::iterator it = m_pRefData->m_mOutputs.find( pOutput->getDescription() );
            if( it != m_pRefData->m_mOutputs.end() )
            {
                return it->second;
            }
        }
        return 0;
    }
#       endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::OutputSignalGeneratorBlueDevice object.
    explicit OutputSignalGeneratorBlueDevice(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : m_pRefData( 0 )
    {
        m_pRefData = new ReferenceCountedData();
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        ComponentList outputs;
        locator.bindComponent( outputs, "DigitalOutputs" );
        if( outputs.isValid() )
        {
            ComponentIterator it( outputs.hObj() );
            it = it.firstChild();
            while( it.isValid() )
            {
                if( it.isList() )
                {
                    m_pRefData->m_mOutputs.insert( std::pair<std::string, OutputProperties*>( it.name(), new OutputProperties( it ) ) );
                }
                ++it;
            }
        }
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::OutputSignalGeneratorBlueDevice from
    /// and exisiting one.
    explicit OutputSignalGeneratorBlueDevice(
        /// [in] A constant reference to the \b mvIMPACT::acquire::OutputSignalGeneratorBlueDevice object, this object shall be created from
        const OutputSignalGeneratorBlueDevice& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor.
    ~OutputSignalGeneratorBlueDevice()
    {
        dealloc();
    }
#       ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::OutputSignalGeneratorBlueDevice objects
    OutputSignalGeneratorBlueDevice& operator=( const OutputSignalGeneratorBlueDevice& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
        }
        return *this;
    }
#       endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Removes the signal definition from the specified output pin.
    /**
     *  Whenever a signal has been defined for a a certain digital output
     *  pin, this pin can no longer controlled using the corresponding functions
     *  in the class \b mvIMPACT::acquire::DigitalOutput. This function removes the
     *  signal definition and hands back manual control of that digital output to the user.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int undefineSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p )
        {
            p->mode.write( ddomManual );
            return DMR_NO_ERROR;
        }
        return DMR_FEATURE_NOT_AVAILABLE;
    }
    /// \brief Checks whether the specified output can be used to create an inverted expose active signal.
    bool canCreateExposeActiveSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        if( !p || !p->mode.isValid() )
        {
            return false;
        }
        unsigned int dictSize = p->mode.dictSize();
        for( unsigned int i = 0; i < dictSize; i++ )
        {
            if( p->mode.getTranslationDictValue( i ) == ddomExposureActive )
            {
                return true;
            }
        }
        return false;
    }
    /// \brief Checks whether the specified output can be used to create an inverted version of a certain signal.
    bool canInvertSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        return ( p && p->inverter.isValid() );
    }
    /// \brief Returns the current mode a digital output is operated in.
    ///
    /// If the mode is not selectable for the current output an exception will be raised.
    TDeviceDigitalOutputMode getOutputMode(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        if( !p )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, DMR_FEATURE_NOT_AVAILABLE, INVALID_ID, "Unsupported feature query(Could not obtain pointer to output properties)" );
        }
        return p->mode.read();
    }
    /// \brief Checks wheter the specified output is currently operated in a certain mode
    bool isOutputModeActive(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The mode the specified output shall be checked for.
        TDeviceDigitalOutputMode mode ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        return ( p && p->mode.isValid() && ( p->mode.read() == mode ) );
    }
    /// \brief Checks whether the specified output will currently invert a certain internal signal.
    bool isSignalInverted(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        return ( p && p->mode.isValid() && p->inverter.isValid() && ( p->inverter.read() == bTrue ) );
    }
    /// \brief This function will configure the digital output to operate in a certain mode.
    /**
     *  After calling this function the digital output will be operated in the mode specified by
     *  \a mode.
     *
     *  Valid values for \a constant are defined by the enum \b mvIMPACT::acquire::TDeviceDigitalOutputMode.
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int setOutputMode(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemCommon
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The mode the specified output shall be operated in. If the mode passed to the
        /// function is not available for the output or the device an exception will be raised.
        TDeviceDigitalOutputMode mode,
        /// [in] Set this parameter to true, to invert the exposure signal
        bool boInverted = false )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p && p->mode.isValid() )
        {
            p->mode.write( mode );
            if( p->inverter.isValid() )
            {
                p->inverter.write( boInverted ? bTrue : bFalse );
            }
            else if( boInverted )
            {
                return DMR_FEATURE_NOT_AVAILABLE;
            }
            return DMR_NO_ERROR;
        }
        return DMR_FEATURE_NOT_AVAILABLE;
    }
};
#   endif // #ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class to handle the digital inputs and outputs for frame grabber devices(\b Device specific interface layout only).
/**
 *  This class provides a convenient basic access to the digital inputs and
 *  outputs of a frame grabber device.
 *
 *  \note
 *  The number of digital inputs and outputs available will vary significantly
 *  from device to device. Some devices might not even have digital inputs and/or
 *  digital outputs. Therefore it's crucial to check for available pins before
 *  using them. The corresponding functions to do that can be found in the
 *  base class \b mvIMPACT::acquire::IOSubSystem.
 *
 *  To create more complex digital output related signals the class
 *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber should be used.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class IOSubSystemFrameGrabber : public IOSubSystemCommon
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::IOSubSystemFrameGrabber object.
    explicit IOSubSystemFrameGrabber(   /// A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : IOSubSystemCommon( pDev ), syncOutputMode()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        ComponentIterator itSyncs;
        locator.bindComponent( itSyncs, "SyncOutputs", 0, 1 );
        if( itSyncs.isValid() && itSyncs.firstChild().isValid() )
        {
            locator.bindSearchBase( locator.searchbase_id(), "SyncOutputs" );
            locator.bindComponent( syncOutputMode, "Mode" );
            itSyncs = itSyncs.firstChild();
            while( itSyncs.isValid() )
            {
                if( itSyncs.isList() )
                {
                    std::string name = itSyncs.name();
                    if( name.substr( 0, 3 ) == "VD-" )
                    {
                        registerVDOutput( itSyncs );
                    }
                    else if( name.substr( 0, 3 ) == "HD-" )
                    {
                        registerHDOutput( itSyncs );
                    }
                    else
                    {
                        ; // type not recognized?!
                    }
                }
                ++itSyncs;
            }
        }
    }
    /// \brief Returns the number of available HD output pins for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int getHDOutputCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_vHDOutputs.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::SyncOutput object that represents a HD output for this device.
    /**
     *  \a nr \b MUST be a valid number ( larger or equal than 0 and smaller than the
     *  value returned by \b mvIMPACT::acquire::IOSubSystemFrameGrabber::getHDOutputCount.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A pointer to a \b mvIMPACT::acquire::SyncOutput object.
     */
    SyncOutput* HDOutput(
        /// [in] The number of the HD output to return.
        unsigned int nr ) const
    {
        return m_pRefData->m_vHDOutputs.at( nr );
    }
    /// \brief Returns the number of available VD output pins for the \b mvIMPACT::acquire::Device associated with this object.
    unsigned int getVDOutputCount( void ) const
    {
        return static_cast<unsigned int>( m_pRefData->m_vVDOutputs.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::SyncOutput object that represents a VD output for this device.
    /**
     *  \a nr \b MUST be a valid number ( larger or equal than 0 and smaller than the
     *  value returned by \b mvIMPACT::acquire::IOSubSystemFrameGrabber::getVDOutputCount.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A pointer to a \b mvIMPACT::acquire::SyncOutput object.
     */
    SyncOutput* VDOutput(
        /// [in] The number of the VD output to return.
        unsigned int nr ) const
    {
        return m_pRefData->m_vVDOutputs.at( nr );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the current mode for sync. signal creation.
    /**
     *  Sometimes it's desired to create a signal to sync. cameras connected to a capture device.
     *  E.g. when capturing 3 monochrome cameras in parallel these should be synchronized. Therefore
     *  some devices can create HD and VD signals that synchronize the connected cameras.
     *
     *  To create snyc. signals this property must be set either to \b mvIMPACT::acquire::dsomNonInterlaced
     *  or \b mvIMPACT::acquire::dsomInterlaced.
     *
     *  In interlaced mode only the frequency of the VD signals can be modified, while in non-interlaced
     *  mode the width (in percent) the signal stays low can be modified as well.
     *
     *  Now e.g. to sync. two or more CCIR compliant cameras connected to a capture device that can
     *  create sync. signals, the HD signals must be generated with a CCIR compliant frequency.
     *  The function \b mvIMPACT::acquire::IOSubSystemFrameGrabber::HDOutput can be used to access the
     *  desired output pin. Then its property \b mvIMPACT::acquire::SyncOutput::frequency_Hz is
     *  set to 15625 Hz (CCIR standard). The remaining values can be left untouched. The capture device
     *  will no generate HD signals with the desired frequency and VD outputs with the frequency
     *  stored by the property \b mvIMPACT::acquire::SyncOutput::frequency_Hz belonging to the
     *  pointer to the object returned by a call to \b mvIMPACT::acquire::IOSubSystemFrameGrabber::VDOutput.
     */
    PropertyIDeviceSyncOutMode syncOutputMode;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIDeviceSyncOutMode getSyncOutputMode( void ) const
    {
        return syncOutputMode;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to create complex digital output signals(\b Device specific interface layout only).
/**
 *  Instances of this class can be used to create more complex digital output
 *  related signals like a single pulse or sequence of pulses on a certain pin
 *  as a reaction on a signal on one of the digital inputs of a device.
 *
 *  The availability of the features provided by this class heavily depends on the
 *  used capture device. Therefore it's crucial to check if a desired feature is available
 *  at all. This class either provides the functions needed for these checks or its
 *  functions will return appropriate error codes.
 *
 *  \note
 *  If the property \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::controlMode
 *  is not available \b NO feature of this class can be used.
 *
 *  \note
 *  If the translation dictionary of \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::controlMode
 *  does not contain \b mvIMPACT::acquire::docmRTC the function
 *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::defineLineScanPulse will \b NOT
 *  be available.
 *
 *  \note
 *  Calling unsupported functions or accessing unsupported features will raise an exception.
 *
 *  Pointers to \b mvIMPACT::acquire::DigitalOutput objects \b MUST be obtained
 *  by an instance to a \b mvIMPACT::acquire::IOSubSystemFrameGrabber object of the
 *  same device.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *  #include <iostream>
 *  #include <vector>
 *
 *  using namespace std;
 *  using namespace mvIMPACT::acquire;
 *
 *  Device* pDev = getValidDevicePointerFromSomewhere();
 *  IOSubSystemFrameGrabber io(pDev);
 *  unsigned int iCnt = io.getInputCount();
 *  cout << "available digital inputs: " << iCnt << endl;
 *  for( unsigned int d=0; d<iCnt; d++ )
 *  {
 *    cout << io.input(d)->getDescription() << endl;
 *  }
 *  unsigned int oCnt = io.getOutputCount();
 *  cout << "available digital outputs: " << oCnt << endl;
 *  // set all available digital outputs to high
 *  for( unsigned int e=0; e<oCnt; e++ )
 *  {
 *    DigitalOutput* pOutput = io.output(e);
 *    cout << "setting " << pOutput->getDescription() << " to high" << endl;
 *    pOutput->set();
 *  }
 *
 *  // check there is at least one digital output
 *  if( oCnt > 0 )
 *  {
 *    OutputSignalGeneratorFrameGrabber osg(pDev);
 *    // check if this device supports enhanced signal creation at all
 *    if( osg.controlMode.isValid() )
 *    {
 *      // define a simple pulse on the first digital output detected.
 *      // this pulse will be low after 100 us for
 *      // 200us and then switches back to high.
 *      osg.definePulse( io.output( 0 ), 0, 100, 200 );
 *
 *      // define a more complex pulse
 *      vector<int> v;
 *      v.push_back(100);
 *      v.push_back(299);
 *      v.push_back(666);
 *      osg.definePulseSequence( io.output( 0 ), 0, v );
 *      osg.controlMode.write( docmSoftware );
 *      osg.imageTrigger.write( ditAfterDigOutSignals );
 *
 *      // check if line scan pulse creation is supported by this device
 *      vector<pair<string, TDigitalOutputControlMode> > dict;
 *      osg.controlMode.getTranslationDict( dict );
 *      unsigned int dictSize = osg.controlMode.dictSize();
 *      for( unsigned int i=0; i<dictSize; i++ )
 *      {
 *        if( dict[i].second == docmRTC )
 *        {
 *          // supported -> define some signal
 *          cout << "checkGrabberInterface: This device supports RTC control signal creation." << endl;
 *          osg.controlMode.write( docmRTC );
 *          osg.defineLineScanPulse( io.output( 0 ), dsosePeriodically, 100, 100, 1, 1 );
 *        }
 *      }
 *    }
 *  }
 * \endcode
 *  \endif
 *
 *  Whenever a signal has been defined for a a certain digital output
 *  pin, this pin can no longer controlled using the corresponding functions
 *  in the class \b mvIMPACT::acquire::DigitalOutput. In order to do that
 *  the pin in question must be released using the function
 *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::undefineSignal.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class OutputSignalGeneratorFrameGrabber
//-----------------------------------------------------------------------------
{
#       ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct OutputProperties
            //-----------------------------------------------------------------------------
    {
        PropertyIDeviceDigitalOutputMode mode;
        PropertyI polarity;
        PropertyI delay_us;
        PropertyI width_us;
        PropertyI width_pclk;
        PropertyI startLevel;
        PropertyI polaritySwitchAfter_us;
        PropertyIDigitalSignal digitalSignal;
        PropertyI pulseStartConfiguration;
        OutputProperties( HLIST hOutput ) : mode(), polarity(), delay_us(), width_us(),
            width_pclk(), startLevel(), polaritySwitchAfter_us(), digitalSignal(),
            pulseStartConfiguration()
        {
            ComponentLocator locator( hOutput );
            locator.bindComponent( mode, "Mode" );
            locator.bindComponent( polarity, "Polarity" );
            locator.bindComponent( delay_us, "Delay_us" );
            locator.bindComponent( width_us, "Width_us" );
            locator.bindComponent( width_pclk, "Width_pclk" );
            locator.bindComponent( startLevel, "StartLevel" );
            locator.bindComponent( polaritySwitchAfter_us, "PolaritySwitchAfter_us" );
            locator.bindComponent( digitalSignal, "DigitalSignal" );
            locator.bindComponent( pulseStartConfiguration, "PulseStartConfiguration" );
        }
    };
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        ComponentList                            m_outputs;
        PropertyIDeviceSignalOutputStartEvent    m_pulseStartEventLineScan;
        PropertyI                                m_softwareSignalPeriod_pclk;
        PropertyI                                m_output;
        PropertyI                                m_width_pclk;
        PropertyI                                m_polarity;
        PropertyI                                m_divider;
        std::map<std::string, OutputProperties*> m_mOutputs;
        unsigned int                             m_refCnt;
        ReferenceCountedData() : m_outputs(), m_pulseStartEventLineScan(),
            m_softwareSignalPeriod_pclk(), m_output(), m_width_pclk(),
            m_polarity(), m_divider(), m_mOutputs(), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            std::map<std::string, OutputProperties*>::iterator itEnd = m_mOutputs.end();
            std::map<std::string, OutputProperties*>::iterator it = m_mOutputs.begin();
            while( it != itEnd )
            {
                delete it->second;
                ++it;
            }
        }
    }* m_pRefData;
    //-----------------------------------------------------------------------------
    void bindPublicProperties( HLIST hList )
    //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( hList );
        locator.bindComponent( controlMode, "ControlMode" );
        locator.bindComponent( pulseStartEvent, "PulseStartEvent" );
        locator.bindComponent( imageTrigger, "ImageTrigger" );
    }
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    OutputProperties* getOutputProperties( DigitalOutput* pOutput ) const
    //-----------------------------------------------------------------------------
    {
        if( pOutput )
        {
            std::map<std::string, OutputProperties*>::iterator it = m_pRefData->m_mOutputs.find( pOutput->getDescription() );
            if( it != m_pRefData->m_mOutputs.end() )
            {
                return it->second;
            }
        }
        return 0;
    }
#       endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber object.
    explicit OutputSignalGeneratorFrameGrabber( /// A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : m_pRefData( 0 ), controlMode(), pulseStartEvent(), imageTrigger()
    {
        m_pRefData = new ReferenceCountedData();
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        locator.bindComponent( m_pRefData->m_outputs, "DigitalOutputs" );
        locator.bindSearchBase( m_pRefData->m_outputs.hObj() );
        bindPublicProperties( m_pRefData->m_outputs.hObj() );
        locator.bindComponent( m_pRefData->m_pulseStartEventLineScan, "PulseStartEvent(LineScan)" );
        locator.bindComponent( m_pRefData->m_softwareSignalPeriod_pclk, "SoftwareSignalPeriod_pclk" );
        locator.bindComponent( m_pRefData->m_output, "Output" );
        locator.bindComponent( m_pRefData->m_width_pclk, "Width_pclk" );
        locator.bindComponent( m_pRefData->m_polarity, "Polarity" );
        locator.bindComponent( m_pRefData->m_divider, "Divider" );
        if( m_pRefData->m_outputs.isValid() )
        {
            ComponentIterator it( m_pRefData->m_outputs.hObj() );
            it = it.firstChild();
            while( it.isValid() )
            {
                if( it.isList() )
                {
                    m_pRefData->m_mOutputs.insert( std::pair<std::string, OutputProperties*>( it.name(), new OutputProperties( it ) ) );
                }
                ++it;
            }
        }
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber from and exisiting one.
    explicit OutputSignalGeneratorFrameGrabber(
        /// [in] A constant reference to the \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber object, this object shall be created from
        const OutputSignalGeneratorFrameGrabber& src ) : m_pRefData( src.m_pRefData ),
        controlMode( src.controlMode ), pulseStartEvent( src.pulseStartEvent ), imageTrigger( src.imageTrigger )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor.
    ~OutputSignalGeneratorFrameGrabber()
    {
        dealloc();
    }
#       ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber objects
    OutputSignalGeneratorFrameGrabber& operator=( const OutputSignalGeneratorFrameGrabber& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
            bindPublicProperties( m_pRefData->m_outputs.hObj() );
        }
        return *this;
    }
#       endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Assigns a single pulse to a digital output.
    /**
     *  Whenever a single pulse of a certain duration and polarity shall be generated
     *  on a digital output as a reaction on a trigger signal that was either generated by one of
     *  digital inputs of the device or software this is the right function to use.
     *
     *  When the signal is generated can be defined by the properties
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::pulseStartEvent and
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::imageTrigger.
     *
     *  \sa
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::undefineSignal
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int definePulse(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The polarity of the pulse to generate.
        ///
        /// - 0: The pulse will be low for \a width_pclk and high otherwise
        /// - 1: The pulse will be high for \a width_pclk and high otherwise
        int polarity,
        /// [in] The delay after the trigger event before a signal generation starts
        int delay,
        /// [in] The width of the pulse to generate
        int width,
        /// [in] The pulse start configuration to be used for this pulse sequence.
        /// Not every device will support the use of pulse start configurations. Valid objects that
        /// can be passed to this function can be obtained from a call to \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfiguration,
        /// and \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfigurationCount. If objects of these type are
        /// not supported, pass 0 or ignor this parameter. In that case the property
        /// \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::imageTrigger will define the start condition
        /// for the signal output.
        PulseStartConfiguration* pPulseStartConfiguration = 0 )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p )
        {
            p->mode.write( ddomPulse );
            p->polarity.write( polarity );
            p->delay_us.write( delay );
            p->width_us.write( width );
            if( pPulseStartConfiguration )
            {
                p->pulseStartConfiguration.write( pPulseStartConfiguration->number() );
            }
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
    /// \brief Assigns a complex signal to a digital output.
    /**
     *  This function can be used when a complex signal shall be generated
     *  on a digital output as a reaction on a trigger signal that was either generated by one of
     *  digital inputs of the device or software.
     *
     *  When the signal is generated can be defined by the properties
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::pulseStartEvent and
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::imageTrigger.
     *
     *  \sa
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::undefineSignal
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int definePulseSequence(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The start level of the generated signal.
        ///
        /// - 0: The signal will start with 'low'
        /// - 1: The signal will start with 'high'
        int startLevel,
        /// [in] An array of integer values. Each values defines a switch in polarity
        /// of the signal after 'x' us.
        /// An array containing '1000', '2000' therefore will describe a signal
        /// starting with \a startLevel, switches polarity after 1000 us and then
        /// again switches polarity after 2000 us.
        const std::vector<int>& sequence,
        /// [in] The pulse start configuration to be used for this pulse sequence.
        /// Not every device will support the use of pulse start configurations. Valid objects that
        /// can be passed to this function can be obtained from a call to \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfiguration,
        /// and \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfigurationCount. If objects of these type are
        /// not supported, pass 0 or ignor this parameter. In that case the property
        /// \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::imageTrigger will define the start condition
        /// for the signal output.
        PulseStartConfiguration* pPulseStartConfiguration = 0 )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p )
        {
            p->mode.write( ddomUser );
            p->startLevel.write( startLevel );
            p->polaritySwitchAfter_us.write( sequence );
            if( pPulseStartConfiguration )
            {
                p->pulseStartConfiguration.write( pPulseStartConfiguration->number() );
            }
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
#       ifndef WRAP_DOTNET
    /// \brief Receives a list of valid signals that can be passed directly to a certain digital output pin.
    /**
     *  Whenever it is necessary to output a certain signal (e.g. a digital input signal connected to the device or
     *  an internal digital signal like e.g. a frame valid signal) to a digital output of the device this can be done
     *  by a call to the function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::definePassThroughSignal
     *
     *  As the type of signals that a device can send to a digital output depents heavily on the device and even on the
     *  digital output as well, this function will return a list of strings and a numerical representation for each signal
     *  that can be sent to this digital output.
     *
     *  \note
     *  This function is much more efficient than calling
     *  \b OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalValue and
     *  \b OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalString and
     *  therefore this function should be called whenever all entries are required.
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int getValidSignalsForPassThroughModes(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [out] A reference to a vector that will receive the list of allowed digital signals
        /// for this particular digital output pin.
        std::vector<std::pair<std::string, TDigitalSignal> >& sequence ) const
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p && p->digitalSignal.isValid() && p->digitalSignal.hasDict() )
        {
            p->digitalSignal.getTranslationDict( sequence );
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
#       endif // WRAP_DOTNET
    /// \brief Returns the number of signals that can be used for pass through signal definitions for a certain digital output pin of this device.
    unsigned int getValidPassThroughSignalCount(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        if( p && p->digitalSignal.isValid() && p->digitalSignal.hasDict() )
        {
            return p->digitalSignal.dictSize();
        }
        return 0;
    }
    /// \brief Receives a valid signal that can be passed directly to a certain digital output pin.
    /**
     *  Whenever it is necessary to output a certain signal (e.g. a digital input signal connected to the device or
     *  an internal digital signal like e.g. a frame valid signal) to a digital output of the device this can be done
     *  by a call to the function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::definePassThroughSignal
     *
     *  As the type of signals that a device can send to a digital output depents heavily on the device and even on the
     *  digital output as well, this function together with the function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalString
     *  can be used to query a list of strings and a numerical representation for each signal
     *  that can be sent to this digital output.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidSignalsForPassThroughModes
     *  to obtain this information.
     *  \endif
     */
    TDigitalSignal getValidPassThroughSignalValue(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The index of the entry to read from the property. The find out the last valid value for this parameter
        /// call \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalCount
        int index = 0 ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        if( !p || !p->digitalSignal.isValid() || !p->digitalSignal.hasDict() )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, DMR_FEATURE_NOT_AVAILABLE, ( p && p->digitalSignal.isValid() ) ? p->digitalSignal.hObj() : INVALID_ID, "Unsupported feature query" );
        }
        return p->digitalSignal.getTranslationDictValue( index );
    }
    /// \brief Receives a valid string representation of a signal that can be passed directly to a certain digital output pin.
    /**
     *  Whenever it is necessary to output a certain signal (e.g. a digital input signal connected to the device or
     *  an internal digital signal like e.g. a frame valid signal) to a digital output of the device this can be done
     *  by a call to the function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::definePassThroughSignal
     *
     *  As the type of signals that a device can send to a digital output depents heavily on the device and even on the
     *  digital output as well, this function together with the function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalString
     *  can be used to query a list of strings and a numerical representation for each signal
     *  that can be sent to this digital output.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidSignalsForPassThroughModes
     *  to obtain this information.
     *  \endif
     */
    std::string getValidPassThroughSignalString(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The index of the entry to read from the property. The find out the last valid value for this parameter
        /// call \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidPassThroughSignalCount
        int index = 0 ) const
    {
        OutputProperties* p = getOutputProperties( pOutput );
        if( !p || !p->digitalSignal.isValid() || !p->digitalSignal.hasDict() )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, DMR_FEATURE_NOT_AVAILABLE, ( p && p->digitalSignal.isValid() ) ? p->digitalSignal.hObj() : INVALID_ID, "Unsupported feature query" );
        }
        return p->digitalSignal.getTranslationDictString( index );
    }
    /// \brief Assing a certain signal to a digital output.
    /**
     *  Some digital input signals or even some internal signals can be passed directly to one or more
     *  digital outputs.
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int definePassThroughSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The digital signal that shall be passed to the digital output.
        /// Valid values for this parameter will be obtained by a call to the function
        /// \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidSignalsForPassThroughModes
        TDigitalSignal digitalSignal,
        /// [in] If set to \a true, the signal will be inverted internally before being passed to the output.
        bool boInverted = false )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p && p->digitalSignal.isValid() )
        {
            p->mode.write( boInverted ? ddomDigitalSignalPassThroughInv : ddomDigitalSignalPassThrough );
            p->digitalSignal.write( digitalSignal );
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
    /// \brief Assing a certain signal to a digital output.
    /**
     *  Some digital input signals or even some internal signals can be passed directly to one or more
     *  digital outputs.
     *
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int definePassThroughSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] The string representation of the digital output that shall be passed to the digital output.
        /// Valid values for this parameter will be obtained by a call to the function
        /// \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::getValidSignalsForPassThroughModes
        const std::string& digitalSignal,
        /// [in] If set to \a true, the signal will be inverted internally before being passed to the output.
        bool boInverted = false )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p && p->digitalSignal.isValid() )
        {
            p->mode.write( boInverted ? ddomDigitalSignalPassThroughInv : ddomDigitalSignalPassThrough );
            p->digitalSignal.writeS( digitalSignal );
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
    /// \brief Removes the signal definition from the specified output pin.
    /**
     *  Whenever a signal has been defined for a a certain digital output
     *  pin, this pin can no longer controlled using the corresponding functions
     *  in the class \b mvIMPACT::acquire::DigitalOutput. This function removes the
     *  signal definition and hands back manual control of that digital output to the user.
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int undefineSignal(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p )
        {
            p->mode.write( ddomManual );
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
    /// \brief Defines a pulse to trigger line scan cameras.
    /**
     *  Calling this function without setting \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::controlMode
     *  to \b mvIMPACT::acquire::docmRTC will have \b NO effect.
     *
     *  \note
     *  Not every frame grabber will offer the \b mvIMPACT::acquire::docmRTC
     *  mode. Check is the mode is available by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDictValue. The detailed description
     *  of this class might contain source code examples for the language you are working with.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     *
     *  \sa
     *  \b mvIMPACT::acquire::OutputSignalGeneratorFrameGrabber::undefineSignal
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR otherwise.
     */
    int defineLineScanPulse(
        /// [in] A pointer to a \b mvIMPACT::acquire::DigitalOutput object
        /// obtained from an instance of the class \b mvIMPACT::acquire::IOSubSystemFrameGrabber
        /// for this device.
        DigitalOutput* pOutput,
        /// [in] Defines the start event for the signal generation. Valid values for this parameter
        /// are defined by \b mvIMPACT::acquire::TDeviceSignalOutputStartEvent.
        TDeviceSignalOutputStartEvent pulseStartEvent,
        /// [in] Defines the frequency for a continuously generated signal if
        /// \a pulseStartEvent is \b mvIMPACT::acquire::dsoseSyncInRisingEdge
        /// or \b mvIMPACT::acquire::dsoseSyncInFallingEdge and is
        /// ignored otherwise.
        int softwareSignalPeriod_pclk,
        /// [in] The width of the signal to generate if \a pulseStartEvent is \b mvIMPACT::acquire::dsoseSyncInRisingEdge,
        /// \b mvIMPACT::acquire::dsosePeriodically or
        /// \b mvIMPACT::acquire::dsoseSyncInFallingEdge and is
        /// ignored otherwise.
        int width_pclk,
        /// [in] The polarity of the signal to generate if \a pulseStartEvent is \b mvIMPACT::acquire::dsoseSyncInRisingEdge,
        /// \b mvIMPACT::acquire::dsosePeriodically or
        /// \b mvIMPACT::acquire::dsoseSyncInFallingEdge and is
        /// ignored otherwise.
        ///
        /// - 0: The signal will be low for \a width_pclk and high otherwise
        /// - 1: The signal will be high for \a width_pclk and high otherwise
        int polarity,
        /// [in] Defines the divider value if \a pulseStartEvent is
        /// \b mvIMPACT::acquire::dsosePeriodically
        unsigned int divider )
    {
        if( !pOutput )
        {
            return DMR_INVALID_PARAMETER;
        }
        OutputProperties* p = getOutputProperties( pOutput );
        if( p )
        {
            m_pRefData->m_pulseStartEventLineScan.write( pulseStartEvent );
            m_pRefData->m_output.writeS( pOutput->getDescription() );
            switch( pulseStartEvent )
            {
            case dsoseSyncInRisingEdge:
            case dsoseSyncInFallingEdge:
                m_pRefData->m_width_pclk.write( width_pclk );
                m_pRefData->m_polarity.write( polarity );
                m_pRefData->m_divider.write( divider );
                break;
            case dsosePeriodically:
                m_pRefData->m_softwareSignalPeriod_pclk.write( softwareSignalPeriod_pclk );
                m_pRefData->m_width_pclk.write( width_pclk );
                m_pRefData->m_polarity.write( polarity );
                break;
            default:
                // nothing to do
                break;
            }
            return DMR_NO_ERROR;
        }
        else
        {
            return DMR_FEATURE_NOT_AVAILABLE;
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the general method used to create output signals.
    /**
     *  This setting always applies to every signal defined. Modifying this property will also
     *  change the behaviour of signals already defined.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigitalOutputControlMode.
     *
     *  When switching to \b mvIMPACT::acquire::docmRTC mode
     *  only one signal can be created currently.
     *
     *  \note
     *  Not every frame grabber will offer the \b mvIMPACT::acquire::docmRTC
     *  mode. Check if the mode is available by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyIDigitalOutputControlMode::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     *
     *  Some devices will offer a more flexible way to define this behaviour. In this case, this property won't be available (a call to the function
     *  \b mvIMPACT::acquire::Component::isValid will return false). However then calls to \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfigurationCount
     *  and \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfiguration will succeed and the objects returned by
     *  \b mvIMPACT::acquire::IOSubSystem::getPulseStartConfiguration (\b mvIMPACT::acquire::PulseStartConfiguration) will
     *  provide a more sofisticated way to define the signal output behaviour of a device.
     */
    PropertyIDigitalOutputControlMode controlMode;
    /// \brief An enumerated integer property defining the general behaviour of created output signals.
    /**
     *  This setting always applies to every signal defined. Modifying this property will also
     *  change the behaviour of signals already defined.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceSignalOutputStartEvent.
     */
    PropertyIDeviceSignalOutputStartEvent pulseStartEvent;
    /// \brief An enumerated integer property defining the general behaviour of created output signals.
    /**
     *  This setting always applies to every signal defined. Modifying this property will also
     *  change the behaviour of signals already defined.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceImageTrigger.
     */
    PropertyIDeviceImageTrigger imageTrigger;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyIDigitalOutputControlMode getControlMode( void ) const
    {
        return controlMode;
    }
    PropertyIDeviceSignalOutputStartEvent getPulseStartEvent( void ) const
    {
        return pulseStartEvent;
    }
    PropertyIDeviceImageTrigger getImageTrigger( void ) const
    {
        return imageTrigger;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class to handle the digital inputs and outputs of a \b mvBlueCOUGAR of \b mvBlueLYNX-M7 device(\b Device specific interface layout only).
/**
 *  This class provides a convenient basic access to the digital inputs and
 *  outputs of a \b mvBlueCOUGAR or \b mvBlueLYNX-M7 device.
 *
 *  \note
 *  The number of digital inputs and outputs available will vary significantly
 *  from device to device. Some devices might not even have digital inputs and/or
 *  digital outputs. Therefore it's crucial to check for available pins before
 *  using them. The corresponding functions to do that can be found in the
 *  base class \b mvIMPACT::acquire::IOSubSystem.
 *
 *  Please also see the code sample in the base class \b mvIMPACT::acquire::IOSubSystemCommon.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class IOSubSystemBlueCOUGAR : public IOSubSystemCommon
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::IOSubSystemBlueCOUGAR object.
    explicit IOSubSystemBlueCOUGAR(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : IOSubSystemCommon( pDev )
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        locator.bindComponent( digitalInputThreshold_mV, "DigitalInputThreshold_mV" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the threshold for the digital inputs in milli-Volt.
    /**
     *  If a voltage applied to the digital input lies above the threshold this pin
     *  will be considered as 'logic 1' otherwise it will be considered as 'logic 0'.
     *
     *  \note
     *  This property is currently only available for \b mvBlueLYNX-M7 devices. Thus always call \b mvIMPACT::acquire::Component::isValid
     *  to check if the device you are working it supports this feature. Not doing so and using the feature with a device
     *  not supporting it will raise an exception.
     */
    PropertyI digitalInputThreshold_mV;
    PYTHON_ONLY( %mutable; )
#ifdef DOTNET_ONLY_CODE
    PropertyI getDigitalInputThreshold_mV( void ) const
    {
        return digitalInputThreshold_mV;
    }
#endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION

//-----------------------------------------------------------------------------
/// \brief A class describing how a video signal source(e.g. a camera or image sensor) is connected to a video signal sink(e.g. a frame grabber)(\b Device specific interface layout only).
/**
 *  With an instance of this class the user can define on which input channels the
 *  video signal source used for this setting is connected to the video signal sink and how(in which format) the
 *  video signal source transmits its data.
 *
 *  Video signal source and video signal sink can both belong to the same physical piece
 *  of hardware. This e.g. might apply to a digital camera that doesn't need a
 *  frame grabber (e.g. a GigE Vision&trade; or USB camera). In such a scenario
 *  certain properties belonging to this class might be read-only or may only allow
 *  a single enumeration value.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class Connector : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::Connector object
    explicit Connector(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : ComponentCollection( pDev ), cameraOutputUsed(),
        videoChannel(), pinDescription()
    {
        DeviceComponentLocator locator( pDev, dltSetting, settingName );
        if( locator.findComponent( "Connector" ) != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "Connector" );
            m_hRoot = locator.searchbase_id();
            locator.bindComponent( cameraOutputUsed, "CameraOutputUsed" );
            locator.bindComponent( videoChannel, "VideoChannel" );
            locator.bindComponent( pinDescription, "PinDescription" );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the video signal output of the video signal source used for the connection to the video signal sink.
    /**
     *  \note
     *  Not every video signal sink will support every possible value defined in the enumeration
     *  \b mvIMPACT::acquire::TCameraOutput. For example a CameraLink&reg; frame grabber
     *  will not offer to use a SVideo connector. Therefore the translation dictionary of this
     *  property should be used to find out, which connector types are available.
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyICameraOutput cameraOutputUsed;
    /// \brief An integer property containing the video channel the camera belonging to the current setting is connected to.
    /**
     *  The maximum possible video channel might change when the property
     *  \b mvIMPACT::acquire::Connector::cameraOutputUsed is modified as the different
     *  ways to transmit a video signal require more or less video input channels. While e.g. a composite
     *  video signal can be transmitted via a single wire, an RGB signal requires either 3 or 4 wires
     *  (depending on whether the sync. signal is transmitted on a separate wire or not).
     *
     *  The first channel will always have the number 0. The limits (max/min values) can be queried
     *  by calling the function \b mvIMPACT::acquire::PropertyI::getMinValue or
     *  \b mvIMPACT::acquire::PropertyI::getMaxValue.
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyI videoChannel;
    /// \brief A string property \b (read-only) containing a description for the video channel currently selected by the property \b mvIMPACT::acquire::Connector::videoChannel
    /**
     *  The description string contains information about the connector pins belonging to this video
     *  input of the video signal sink as well as the corresponding name of this video input.
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyS pinDescription;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyICameraOutput getCameraOutputUsed( void ) const
    {
        return cameraOutputUsed;
    }
    PropertyI getVideoChannel( void ) const
    {
        return videoChannel;
    }
    PropertyS getPinDescription( void ) const
    {
        return pinDescription;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class for camera related settings(\b Device specific interface layout only).
/**
 * This class acts as a base class for camera related settings. It only contains
 * settings that are available for every device!
 *
 * \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 * \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsBase : public BasicDeviceSettings
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsBase object
    explicit CameraSettingsBase(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : BasicDeviceSettings( pDev, settingName ), aoiHeight(),
        aoiStartX(), aoiStartY(), aoiWidth(), pixelFormat()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( pixelFormat, "PixelFormat" );
        if( locator.findComponent( "Aoi" ) != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "Aoi" );
            locator.bindComponent( aoiHeight, "H" );
            locator.bindComponent( aoiStartX, "X" );
            locator.bindComponent( aoiStartY, "Y" );
            locator.bindComponent( aoiWidth, "W" );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the number of lines to capture.
    PropertyI aoiHeight;
    /// \brief An integer property defining the X-offset for each capture line.
    /**
     *  Example: When setting this property to 5 the first pixel in each line of
     *  the resulting image will be pixel number 5 of each line transmitted by
     *  the camera.
     */
    PropertyI aoiStartX;
    /// \brief An integer property defining the Y-offset.
    /**
     *  Example: When setting this property to 5 the first line of
     *  the resulting image will be line number 5 of the image transmitted by
     *  the camera.
     */
    PropertyI aoiStartY;
    /// \brief An integer property defining the number of pixels to capture per line.
    PropertyI aoiWidth;
    /// \brief An enumerated integer property defining the pixel format used to transfer the image data into the target systems host memory.
    /**
     *  Support for this property has been added in version 1.11.0 of mvIMPACT Acquire thus old driver versions will
     *  raise an exception if an application accesses the property without checking if the property is present.
     *
     *  By default this property will be set to \b mvIMPACT::acquire::ibpfAuto. This will result in the previous driver
     *  behaviour where depending on the various post processing steps that are enabled or disabled the driver will decide
     *  which transfer format will be used. However sometimes this could result in a transfer format, which is not ideal
     *  for the application (e.g. when a post processing step of the application needs RGBx888Packed but the systems bandwidth
     *  is limited the application might want to transfer the data in YUV422 packed and then use the fast format conversion
     *  algorithm in the driver). This can be achieved by explicitly setting \b mvIMPACT::acquire::CameraSettingsBase::pixelFormat to
     *  \b mvIMPACT::acquire::ibpfYUV422Packed and \b mvIMPACT::acquire::ImageDestination::pixelFormat to
     *  \b mvIMPACT::acquire::ibpfRGBx888Packed.
     *
     *  \note
     *  Selecting a defined transfer format can in some cases result in certain filters (e.g. dark current, ...) to be switched
     *  of as then the filters themself can no longer influence the trnasfer format, which is sometimes necessary as not every
     *  filter does support every input format. Also this property will contain only pixel formats, which are actually supported
     *  by the capture device, thus in most of the cases this will be a subset of the pixel formats defined by
     *  \b mvIMPACT::acquire::TImageBufferPixelFormat
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TImageBufferPixelFormat.
     */
    PropertyIImageBufferPixelFormat pixelFormat;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyI getAoiHeight( void ) const
    {
        return aoiHeight;
    }
    PropertyI getAoiStartX( void ) const
    {
        return aoiStartX;
    }
    PropertyI getAoiStartY( void ) const
    {
        return aoiStartY;
    }
    PropertyI getAoiWidth( void ) const
    {
        return aoiWidth;
    }
    PropertyIImageBufferPixelFormat getPixelFormat( void ) const
    {
        return pixelFormat;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

#   ifndef IGNORE_MVVIRTUALDEVICE_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief \b mvVirtualDevice related camera settings(\b Device specific interface layout only).
/**
 *  \note
 *  As this is a device driver meant for testing the interface an allowing to
 *  start the integration of mvIMPACT Acquire into an application, this class
 *  might change at ANY time! Changes in the virtual device driver might not
 *  be backward compatible!
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsVirtualDevice : public CameraSettingsBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsVirtualDevice object
    explicit CameraSettingsVirtualDevice(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBase( pDev, settingName ),
        gain_dB(), testMode(), channelBitDepth(), paddingX(), tapsXGeometry(), tapsYGeometry(),
        frameDelay_us(), imageDirectory(), imageType(), bayerMosaicParity(), testImageBarWidth(), userData()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( gain_dB, "Gain_dB" );
        locator.bindComponent( testMode, "TestMode" );
        locator.bindComponent( channelBitDepth, "ChannelBitDepth" );
        locator.bindComponent( paddingX, "PaddingX" );
        locator.bindComponent( tapsXGeometry, "TapsXGeometry" );
        locator.bindComponent( tapsYGeometry, "TapsYGeometry" );
        locator.bindComponent( frameDelay_us, "FrameDelay_us" );
        locator.bindComponent( imageDirectory, "ImageDirectory" );
        locator.bindComponent( imageType, "ImageType" );
        locator.bindComponent( bayerMosaicParity, "BayerMosaicParity" );
        locator.bindComponent( testImageBarWidth, "TestImageBarWidth" );
        locator.bindComponent( userData, "UserData" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A float property defining the gain in dB to be applied to the test image.
    /**
     * The formula for given \c gain_dB is
     * \code
     * gain_x= 10 ^ (gain_dB/20)
     * \endcode
     *
     * \b Example:
     *
     * \code
     * gain_x = 10 ^ (6/20) = 1.995
     * \endcode
     *
     * I.e. increasing the gain setting by 6dB corresponds to factor of 2.
     *
     * The formula for given gain_x is
     * \code
     * gain_dB = 20*log(gain_x) dB
     * \endcode
     *
     * Where: \n
     * \c gain_dB: MATRIX VISION gain value (logarithmic) \n
     * \c gain_x: multiplicative gain \n
     * \c ^: power function \n
     */
    PropertyF gain_dB;
    /// \brief An enumerated integer property defining the test mode use to generate the dummy images.
    /**
     *  When \b mvIMPACT::acquire::CameraSettingsBase::pixelFormat is \b NOT set to \b mvIMPACT::acquire::ibpfAuto, this
     *  this property will become invisible.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TVirtualDeviceTestMode.
     */
    PropertyIVirtualDeviceTestMode testMode;
    /// \brief An integer property defining the channel bit depth for certain test image generation modes.
    /**
     *  When \b mvIMPACT::acquire::CameraSettingsBase::pixelFormat is \b NOT set to \b mvIMPACT::acquire::ibpfAuto, this
     *  this property will become invisible.
     *
     *  When \b mvIMPACT::acquire::CameraSettingsVirtualDevice::testMode is set to \b mvIMPACT::acquire::vdtmMovingMonoRamp,
     *  \b mvIMPACT::acquire::vdtmMovingBayerDataRamp or \b mvIMPACT::acquire::vdtmBayerWhiteBalanceTestImage this property
     *  will allow to define the pixel format for the test image.
     */
    PropertyI channelBitDepth;
    /// \brief An integer property defining the padding(in bytes) in X-direction for certain test image generation modes.
    /**
     *  When \b mvIMPACT::acquire::CameraSettingsVirtualDevice::testMode is set to a mono format(not the weird packed ones!),
     *  \b mvIMPACT::acquire::vdtmMovingBayerDataRamp or \b mvIMPACT::acquire::vdtmBayerWhiteBalanceTestImage this property
     *  will allow to define the padding in bytes for each line. This will result in images where the line pitch differs from
     *  the width multiplied by the bytes per pixel value. E.g. a padding of 1 in X-direction and a width of 5 for a 16 bit mono
     *  format will result in a line pitch of 11 bytes(5*2 + 1).
     */
    PropertyI paddingX;
    /// \brief An enumerated integer property defining the way this camera transmits the pixel data in X direction.
    /**
     *  Within the virtual device driver, modifying this property internally will only attach the corresponding
     *  attribute to the request buffer, thus might result in the tap sort filter to process the data. This is mainly
     *  meant for testing purposes.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTapsXGeometry.
     */
    PropertyICameraTapsXGeometry tapsXGeometry;
    /// \brief An enumerated integer property defining the way this camera transmits the pixel data in Y direction.
    /**
     *  Within the virtual device driver, modifying this property internally will only attach the corresponding
     *  attribute to the request buffer, thus might result in the tap sort filter to process the data. This is mainly
     *  meant for testing purposes.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTapsYGeometry.
     */
    PropertyICameraTapsYGeometry tapsYGeometry;
    /// \brief An integer property defining a delay in us before the 'captured' image is returned to the user
    PropertyI frameDelay_us;
    /// \brief A string property defining the directory to capture images from.
    /**
     *  This feature will only be visible, when the property \b mvIMPACT::acquire::CameraSettingsVirtualDevice::testMode
     *  is set to \b mvIMPACT::acquire::vdtmImageDirectory.
     */
    PropertyS imageDirectory;
    /// \brief An enumerated integer property defining what images shall be captured from harddisc.
    /**
     *  This feature will only be visible, when the property \b mvIMPACT::acquire::CameraSettingsVirtualDevice::testMode
     *  is set to \b mvIMPACT::acquire::vdtmImageDirectory.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TVirtualDeviceTestMode.
     */
    PropertyIVirtualDeviceImageType imageType;
    /// \brief An enumerated integer property defining the bayer attribute assigned to the generated test image.
    /**
     *  When images are acquired via a directory, or a bayer test pattern is generated this can be used to specify the bayer parity.
     *  \b mvIMPACT::acquire::bmpUndefined will set the buffers bayer attribute thus will result in a grey(mono) buffer being returned.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBayerMosaicParity.
     */
    PropertyIBayerMosaicParity bayerMosaicParity;
    /// \brief An integer property defining the width (in pixels) of the different bars in certain test modes.
    PropertyI testImageBarWidth;
    /// \brief A string property that will be copied into the user data property of each request.
    /**
     *  This e.g. can be used to assign a certain identifier to each image request.
     */
    PropertyS userData;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyF getGain_dB( void ) const
    {
        return gain_dB;
    }
    PropertyIVirtualDeviceTestMode getTestMode( void ) const
    {
        return testMode;
    }
    PropertyI getChannelBitDepth( void ) const
    {
        return channelBitDepth;
    }
    PropertyI getPaddingX( void ) const
    {
        return paddingX;
    }
    PropertyICameraTapsXGeometry getTapsXGeometry( void ) const
    {
        return tapsXGeometry;
    }
    PropertyICameraTapsYGeometry getTapsYGeometry( void ) const
    {
        return tapsYGeometry;
    }
    PropertyI getFrameDelay_us( void ) const
    {
        return frameDelay_us;
    }
    PropertyS getImageDirectory( void ) const
    {
        return imageDirectory;
    }
    PropertyIVirtualDeviceImageType getImageType( void ) const
    {
        return imageType;
    }
    PropertyI getTestImageBarWidth( void ) const
    {
        return testImageBarWidth;
    }
    PropertyIBayerMosaicParity getBayerMosaicParity( void ) const
    {
        return bayerMosaicParity;
    }
    PropertyS getUserData( void ) const
    {
        return userData;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVVIRTUALDEVICE_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVV4L2_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief \b mvV4L2 related camera settings(\b Device specific interface layout only).
/**
 *  \note UNDER CONSTRUCTION! Subject to change.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsV4L2Device : public CameraSettingsBase
//-----------------------------------------------------------------------------
{
    ComponentIterator        m_customFeatureIterator;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsV4L2Device object.
    explicit CameraSettingsV4L2Device(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBase( pDev, settingName ),
        m_customFeatureIterator(), imageWidth(), imageHeight(), videoStandard(), pixelFormat(),
        brightness(), contrast(), saturation(), hue(), blackLevel(), autoWhiteBalance(), redBalance(),
        blueBalance(), gamma(), exposure(), autoGain(), gain(), HFlip(), VFlip(), powerLineFrequency(),
        hueAuto(), whiteBalanceTemperature(), sharpness(), backlightCompensation()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( imageWidth, "ImageWidth" );
        locator.bindComponent( imageHeight, "ImageHeight" );
        locator.bindComponent( videoStandard, "VideoStandard" );
        locator.bindComponent( pixelFormat, "PixelFormat" );
        locator.bindComponent( brightness, "Brightness" );
        locator.bindComponent( contrast, "Contrast" );
        locator.bindComponent( saturation, "Saturation" );
        locator.bindComponent( hue, "Hue" );
        locator.bindComponent( blackLevel, "BlackLevel" );
        locator.bindComponent( autoWhiteBalance, "AutoWhiteBalance" );
        locator.bindComponent( redBalance, "RedBalance" );
        locator.bindComponent( blueBalance, "BlueBalance" );
        locator.bindComponent( gamma, "Gamma" );
        locator.bindComponent( exposure, "Exposure" );
        locator.bindComponent( autoGain, "AutoGain" );
        locator.bindComponent( gain, "Gain" );
        locator.bindComponent( HFlip, "HFlip" );
        locator.bindComponent( VFlip, "VFlip" );
        locator.bindComponent( powerLineFrequency, "PowerLineFrequency" );
        locator.bindComponent( hueAuto, "HueAuto" );
        locator.bindComponent( whiteBalanceTemperature, "WhiteBalanceTemperature" );
        locator.bindComponent( sharpness, "Sharpness" );
        locator.bindComponent( backlightCompensation, "BacklightCompensation" );
        m_customFeatureIterator = ComponentIterator( pixelFormat.hObj() );
        m_customFeatureIterator = m_customFeatureIterator.firstSibling();
    }
    /// \brief Returns an iterator to for iterating inside a device specific feature list.
    /**
     *  This can be useful a device offers custom features that are not defined in the V4L2 standard and thus
     *  can't be known at compile time.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  It allows to write code like this:
     *
     * \code
     *  CameraSettingsV4L2Device cs(getDevicePointerFromSomewhere());
     *  ComponentIterator it(cs.getCustomFeatureIterator());
     *  std::map<std::string, std::string> m;
     *  while( it.isValid() )
     *  {
     *    // collect all entries that are properties and store their current name and value in a map
     *    if( it.isProp() && it.isVisible() )
     *    {
     *      Property prop(it);
     *      m.insert( std::make_pair( prop.name(), prop.readS() );
     *    }
     *    ++it;
     *  }
     * \endcode
     *  \endif
     */
    ComponentIterator getCustomFeatureIterator( void ) const
    {
        return m_customFeatureIterator;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the width of the image supplied by the V4L2 device.
    /**
     *  \sa V4L2 API at http://v4l2spec.bytesex.org/spec-single/v4l2.html
     */
    PropertyI imageWidth;
    /// \brief An integer property defining the height of the image supplied by the V4L2 device.
    /**
     *  \sa V4L2 API at http://v4l2spec.bytesex.org/spec-single/v4l2.html
     */
    PropertyI imageHeight;
    /// \brief An integer property defining the video standard
    /**
     *  Use this property to configure the V4L2-device corresponding to connectes video source.
     *  Availabale video standards depend on the V4L2-device and conform to \b v4l2_std_id
     *  \sa V4L2 API at http://v4l2spec.bytesex.org/spec-single/v4l2.html
     */
    PropertyI videoStandard;
    /// \brief An integer property defining the pixelformat of captured frames.
    /**
     *  Use this property to set the pixelformat within the image buffer supplied by the V4L2-device.
     *  Available pixelformats depend on the V4L2-device and conform to \b v4l2_fourcc()
     *  \sa V4L2 API at http://v4l2spec.bytesex.org/spec-single/v4l2.html
     */
    PropertyI pixelFormat;
    /// \brief An integer property to adjust the brightness.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI brightness;
    /// \brief An integer property to adjust the contrast.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI contrast;
    /// \brief An integer property to adjust the saturation.
    /**
     *  This property represents a V4L2-control ID.
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI saturation;
    /// \brief An integer property to adjust the hue.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI hue;
    /// \brief An integer property to adjust the black level.
    /**
     *  This property represents a V4L2-control ID.
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI blackLevel;
    /// \brief An enumerated integer property to switch on/off the auto white balance function of the V4L2-device
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean autoWhiteBalance;
    /// \brief An integer property to adjust the red balance.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI redBalance;
    /// \brief An integer property to adjust the blue balance.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI blueBalance;
    /// \brief An integer property to adjust the gamma.
    /**
     *  This property represents a V4L2-control ID
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a min-value, a max-value and step-width are defined. Thus, invalid values may be tuned after writing to fit within the limits.
     */
    PropertyI gamma;
    /// \brief An integer property to adjust the exposure.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI exposure;
    /// \brief An enumerated integer property to switch on/off the auto gain.
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean autoGain;
    /// \brief An integer property to adjust the gain.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI gain;
    /// \brief An enumerated integer property to mirror the frames horizontally
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean HFlip;
    /// \brief An integer property to mirror the frames vertically.
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean VFlip;
    /// \brief An enumerated integer property to enable power line frequency filter.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *
     *  If supported, possible values conform to V4L2 API:
     *  <table>
     *  <tr><td class="header">string representation</td><td class="header">numerical representation</td></tr>
     *  <tr><td class="indexvalue">V4L2_CID_POWER_LINE_FREQUENCY_DISABLED</td><td class="indexvalue">0</td></tr>
     *  <tr><td class="indexvalue">V4L2_CID_POWER_LINE_FREQUENCY_50HZ</td><td class="indexvalue">1</td></tr>
     *  <tr><td class="indexvalue">V4L2_CID_POWER_LINE_FREQUENCY_60HZ</td><td class="indexvalue">2</td></tr>
     *  </table>
     */
    PropertyI powerLineFrequency;
    /// \brief An enumerated integer property to switch on/off auto hue.
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean hueAuto;
    /// \brief An integer property to adjust white balnace temperature.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI whiteBalanceTemperature;
    /// \brief An integer property to adjust the sharpness.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI sharpness;
    /// \brief An integer property to adjust the backlight compensation.
    /**
     *  This property represents a V4L2-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a min-value, a max-value and step-width are defined. Thus, invalid values may be tuned after writing to fit within the limits.
     */
    PropertyI backlightCompensation;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getImageWidth( void ) const
    {
        return imageWidth;
    }
    PropertyI getImageHeight( void ) const
    {
        return imageHeight;
    }
    PropertyI getVideoStandard( void ) const
    {
        return videoStandard;
    }
    PropertyI getPixelFormat( void ) const
    {
        return pixelFormat;
    }
    PropertyI getBrightness( void ) const
    {
        return brightness;
    }
    PropertyI getContrast( void ) const
    {
        return contrast;
    }
    PropertyI getSaturation( void ) const
    {
        return saturation;
    }
    PropertyI getHue( void ) const
    {
        return hue;
    }
    PropertyI getBlackLevel( void ) const
    {
        return blackLevel;
    }
    PropertyIBoolean getAutoWhiteBalance( void ) const
    {
        return autoWhiteBalance;
    }
    PropertyI getRedBalance( void ) const
    {
        return redBalance;
    }
    PropertyI getBlueBalance( void ) const
    {
        return blueBalance;
    }
    PropertyI getGamma( void ) const
    {
        return gamma;
    }
    PropertyI getExposure( void ) const
    {
        return exposure;
    }
    PropertyIBoolean getAutoGain( void ) const
    {
        return autoGain;
    }
    PropertyI getGain( void ) const
    {
        return gain;
    }
    PropertyIBoolean getHFlip( void ) const
    {
        return HFlip;
    }
    PropertyIBoolean getVFlip( void ) const
    {
        return VFlip;
    }
    PropertyI getPowerLineFrequency( void ) const
    {
        return powerLineFrequency;
    }
    PropertyIBoolean getHueAuto( void ) const
    {
        return hueAuto;
    }
    PropertyI getWhiteBalanceTemperature( void ) const
    {
        return whiteBalanceTemperature;
    }
    PropertyI getSharpness( void ) const
    {
        return sharpness;
    }
    PropertyI getBacklightCompensation( void ) const
    {
        return backlightCompensation;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVV4L2_SPECIFIC_DOCUMENTATION

#   if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)
//-----------------------------------------------------------------------------
/// \brief A class to control the automatic control parameters of a device(\b Device specific interface layout only).
/**
 * Currently this class provides access to the properties, which control the way the
 * AGC(\b A utomatic \b G ain \b C ontrol) and the AEC
 * (\b A utomatic \b E xpose \b C ontrol) operates.
 *
 * The values used to capture the image, which are directly influenced by this properties
 * (e.g. the gain or the exposure time) will be returned as part of the result of the
 * image request.
 *
 * \note
 * Not every device will offer the features provided in this class. Before accessing any members
 * and/or properties call the function \b mvIMPACT::acquire::AutoControlParameters::isAvailable.
 * If this function returns false, no other member or function call in this class will return valid
 * results.
 *
 * \note
 * Instances of these class can't be constructed directly. Its parameters can
 * be accessed via an instance of the class \b mvIMPACT::acquire::CameraSettingsBlueDevice.
 *
 * \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 * \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class AutoControlParameters : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class CameraSettingsBlueDevice;
    bool m_boAvailable;
    explicit AutoControlParameters( HOBJ hObj ): m_boAvailable( false ), aoiHeight(),
        aoiStartX(), aoiStartY(), aoiWidth(), aoiMode(), desiredAverageGreyValue(),
        controllerSpeed(), controllerGain(), controllerIntegralTime_ms(), controllerDerivativeTime_ms(),
        controllerDelay_Images(), gainLowerLimit_dB(),
        gainUpperLimit_dB(), exposeLowerLimit_us(), exposeUpperLimit_us()
    {
        ComponentLocator locator( hObj );
        HOBJ hRoot = locator.findComponent( "AutoControlParameters" );
        if( hRoot != INVALID_ID )
        {
            m_hRoot = hRoot;
            locator.bindSearchBase( locator.searchbase_id(), "AutoControlParameters" );
            locator.bindComponent( aoiMode, "AoiMode" );
            locator.bindComponent( desiredAverageGreyValue, "DesiredAverageGreyValue" );
            locator.bindComponent( controllerSpeed, "ControllerSpeed" );
            locator.bindComponent( controllerDelay_Images, "ControllerDelay_Images" );
            locator.bindComponent( gainLowerLimit_dB, "GainLowerLimit_dB" );
            locator.bindComponent( gainUpperLimit_dB, "GainUpperLimit_dB" );
            locator.bindComponent( exposeLowerLimit_us, "ExposeLowerLimit_us" );
            locator.bindComponent( exposeUpperLimit_us, "ExposeUpperLimit_us" );
            if( locator.findComponent( "ControllerParameter" ) != INVALID_ID )
            {
                locator.bindSearchBase( hRoot, "ControllerParameter" );
                locator.bindComponent( controllerGain, "ControllerGain" );
                locator.bindComponent( controllerIntegralTime_ms, "IntegralTime_ms" );
                locator.bindComponent( controllerDerivativeTime_ms, "DerivativeTime_ms" );
            }
            locator.bindSearchBase( hRoot );
            if( locator.findComponent( "Aoi" ) != INVALID_ID )
            {
                locator.bindSearchBase( hRoot, "Aoi" );
                locator.bindComponent( aoiHeight, "H" );
                locator.bindComponent( aoiStartX, "X" );
                locator.bindComponent( aoiStartY, "Y" );
                locator.bindComponent( aoiWidth, "W" );
            }
            m_boAvailable = true;
        }
    }
public:
    /// \brief This function should be called to check if this device offers auto control parameters.
    /**
     *  \return
     *  - true if the device offers the features defined by this object.
     *  - false otherwise
     */
    bool isAvailable( void ) const
    {
        return m_boAvailable;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the height of the rectangle used for the parameter calculation.
    PropertyI aoiHeight;
    /// \brief An integer property defining the X-offset of the rectangle used for the parameter calculation.
    /**
     *  Example: When setting this property to 5 the first pixel in each line of
     *  the resulting rectangle used for the calculation will be pixel number 5
     *  of each line transmitted by the camera.
     */
    PropertyI aoiStartX;
    /// \brief An integer property defining the Y-offset of the rectangle used for the parameter calculation.
    /**
     *  Example: When setting this property to 5 the first line of
     *  the resulting rectangle used for the calculation will be line number 5 of the
     *  image transmitted by the camera.
     */
    PropertyI aoiStartY;
    /// \brief An integer property defining the width of the rectangle.
    PropertyI aoiWidth;
    /// \brief An enumerated integer property defining the which area of the image is used for the calculation of the parameters.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAoiMode.
     */
    PropertyIAoiMode aoiMode;
    /// \brief An integer property defining the average grey value to be reached within the AOI in the image by the control circuit.
    /**
     *  For multi-byte pixel formats like e.g. \b mvIMPACT::acquire::ibpfMono10 this value will refer to
     *  the 8 most significant bits of the pixel data, thus e.g a value of 128 for this property will result
     *  it the controller trying reach an average grey value of 512 (0x200).
     */
    PropertyI desiredAverageGreyValue;
    /// \brief An enumerated integer property defining the speed the control ciruit will be operated in.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAutoControlSpeed.
     *
     *  Once the controller started to react and adapt itself to a new situation, this property will
     *  define how long this adaption will take, while \b AutoControlParameters::controllerDelay_Images
     *  defines the the number of images to wait from a detected change to the actual start of the
     *  adaption to this new situation.
     */
    PropertyIAutoControlSpeed controllerSpeed;
    /// \brief The \b P fraction of the controller.
    /**
     *  This feature will only be visible when \b mvIMPACT::acquire::AutoControlParameters::controllerSpeed
     *  is set to \b mvIMPACT::acquire::acsUserDefined.
     */
    PropertyF controllerGain;
    /// \brief The \b I fraction of the controller.
    /**
     *  This feature will only be visible when \b mvIMPACT::acquire::AutoControlParameters::controllerSpeed
     *  is set to \b mvIMPACT::acquire::acsUserDefined.
     */
    PropertyF controllerIntegralTime_ms;
    /// \brief The \b D fraction of the controller.
    /**
     *  This feature will only be visible when \b mvIMPACT::acquire::AutoControlParameters::controllerSpeed
     *  is set to \b mvIMPACT::acquire::acsUserDefined.
     */
    PropertyF controllerDerivativeTime_ms;
    /// \brief An integer property to influence the speed the AGC and/or AEC control circuit will react and adapt to changes.
    /**
     *  While \b AutoControlParameters::controllerSpeed influences the overall time
     *  needed to adjust the controller to a new situation, this property can be used to define a
     *  delay when the controller shall start to adapt to a changed situation.
     */
    PropertyI controllerDelay_Images;
    /// \brief A float property defining the lower limit for the cameras gain(in dB).
    /**
     *  When the AGC is active this value defines the minimum value for the cameras gain.
     *  Even if the controller can reach the desired average grey value, the gain will
     *  never fall below this value.
     */
    PropertyF gainLowerLimit_dB;
    /// \brief A float property defining the upper limit for the cameras gain(in dB).
    /**
     *  When the AGC is active this value defines the maximum value for the cameras gain.
     *  Even if the controller can reach the desired average grey value, the gain will
     *  never exceed this value.
     */
    PropertyF gainUpperLimit_dB;
    /// \brief A float property defining the lower limit for the sensors expose time(in us).
    /**
     *  When the AEC is active this value defines the minimum value for the sensors expose time(in us).
     *  Even if the controller can reach the desired average grey value, the expose time will
     *  never fall below this value.
     */
    PropertyI exposeLowerLimit_us;
    /// \brief An integer property defining the upper limit for the sensors expose time in us.
    /**
     *  When the AEC is active this value defines the maximum value for the sensors expose time(in us).
     *  Even if the controller can reach the desired average grey value, the expose time will
     *  never exceed this value.
     */
    PropertyI exposeUpperLimit_us;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getAoiHeight( void ) const
    {
        return aoiHeight;
    }
    PropertyI getAoiStartX( void ) const
    {
        return aoiStartX;
    }
    PropertyI getAoiStartY( void ) const
    {
        return aoiStartY;
    }
    PropertyI getAoiWidth( void ) const
    {
        return aoiWidth;
    }
    PropertyIAoiMode getAoiMode( void ) const
    {
        return aoiMode;
    }
    PropertyI getDesiredAverageGreyValue( void ) const
    {
        return desiredAverageGreyValue;
    }
    PropertyIAutoControlSpeed getControllerSpeed( void ) const
    {
        return controllerSpeed;
    }
    PropertyF getControllerGain( void ) const
    {
        return controllerGain;
    }
    PropertyF getControllerIntegralTime_ms( void ) const
    {
        return controllerIntegralTime_ms;
    }
    PropertyF getControllerDerivativeTime_ms( void ) const
    {
        return controllerDerivativeTime_ms;
    }
    PropertyI getControllerDelay_Images( void ) const
    {
        return controllerDelay_Images;
    }
    PropertyF getGainLowerLimit_dB( void ) const
    {
        return gainLowerLimit_dB;
    }
    PropertyF getGainUpperLimit_dB( void ) const
    {
        return gainUpperLimit_dB;
    }
    PropertyI getExposeLowerLimit_us( void ) const
    {
        return exposeLowerLimit_us;
    }
    PropertyI getExposeUpperLimit_us( void ) const
    {
        return exposeUpperLimit_us;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class for camera related settings belonging to the mvBlueXXX-series (\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsBlueDevice : public CameraSettingsBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsBlueDevice object
    explicit CameraSettingsBlueDevice(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBase( pDev, settingName ),
        autoControlMode(), autoGainControl(), autoExposeControl(), gain_dB(),
        offset_pc(), pixelClock_KHz(), exposeMode(), expose_us(),
        binningMode(), testMode(), triggerMode(), frameDelay_us(), autoControlParameters( m_hRoot )
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( autoControlMode, "AutoControlMode" );
        locator.bindComponent( autoGainControl, "AutoGainControl" );
        locator.bindComponent( autoExposeControl, "AutoExposeControl" );
        locator.bindComponent( gain_dB, "Gain_dB" );
        locator.bindComponent( offset_pc, "Offset_pc" );
        locator.bindComponent( pixelClock_KHz, "PixelClock_KHz" );
        locator.bindComponent( exposeMode, "ExposeMode" );
        locator.bindComponent( expose_us, "Expose_us" );
        locator.bindComponent( binningMode, "BinningMode" );
        locator.bindComponent( testMode, "TestMode" );
        locator.bindComponent( triggerMode, "TriggerMode", 0, 0 );
        locator.bindComponent( frameDelay_us, "FrameDelay_us" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the auto control mode the device is operated in.
    /**
     *  This property can be used to control the overall behaviour of the algorithms used for performing AEC
     *  (\b A utomatic \b E xpose \b C ontrol) or AGC (\b A utomatic \b G ain \b C ontrol).
     *  This could be done by the device itself thus e.g. even in the analog domain or e.g. by a software
     *  process in the driver. Each method will offer certain advantages and disadvantages and depending on the
     *  application the correct method can be selected.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAutoControlMode.
     *
     *  Every device or even different models belonging to the same device family may support different auto control modes,
     *  thus it's crucial to query the valid modes for each device by reading the properties translation dictionary BEFORE trying
     *  to assign a certain value to this property.
     */
    PropertyIAutoControlMode autoControlMode;
    /// \brief An enumerated integer property, which represents the current mode the AGC (\b A utomatic \b G ain \b C ontrol) is operated in.
    /**
     *  The values used to capture the image, which are directly influenced by an activated AGC or AEC
     *  (e.g. the gain or the exposure time) will be returned as part of the result of the
     *  image request.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAutoGainControl.
     *
     *  \note
     *  This property is not supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIAutoGainControl autoGainControl;
    /// \brief An enumerated integer property, which represents the current mode the AEC (\b A utomatic \b E xpose \b C ontrol) is operated in.
    /**
     *  The values used to capture the image, which are directly influenced by an activated AGC or AEC
     *  (e.g. the gain or the exposure time) will be returned as part of the result of the
     *  image request.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAutoExposureControl.
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIAutoExposureControl autoExposeControl;
    /// \brief A float property defining the gain in dB.
    /**
     * The formula for given \c gain_dB is
     * \code
     * gain_x= 10 ^ (gain_dB/20)
     * \endcode
     *
     * \b Example:
     *
     * \code
     * gain_x = 10 ^ (6/20) = 1.995
     * \endcode
     *
     * I.e. increasing the gain setting by 6dB corresponds to factor of 2.
     *
     * The formula for given gain_x is
     * \code
     * gain_dB = 20*log(gain_x) dB
     * \endcode
     *
     * Where: \n
     * \c gain_dB: MATRIX VISION gain value (logarithmic) \n
     * \c gain_x: multiplicative gain \n
     * \c ^: power function \n
     */
    PropertyF gain_dB;
    /// \brief A float property defining the analogue sensor offset in percent of the allowed range (sensor specific).
    /**
     *  This property influences the average grey level when no light reaches the sensor.
     *
     */
#       ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
    /** \b mvBlueFOX \b specific:
     *  Changing its value will have no effect if \b mvIMPACT::acquire::CameraSettingsBlueFOX::offsetAutoCalibration is set to
     *  \b mvIMPACT::acquire::aocOn.
     *
     *  By default
     *  this value will be around 10 (if \b mvIMPACT::acquire::CameraSettingsBlueFOX::offsetAutoCalibration if set
     *  to \b mvIMPACT::acquire::aocOn). This means the average grey value for an image
     *  taken in a completely dark environment will be around 10. When setting
     *  \b mvIMPACT::acquire::CameraSettingsBlueFOX::offsetAutoCalibration to \b mvIMPACT::acquire::aocOff
     *  \b mvIMPACT::acquire::CameraSettingsBlueFOX::offset_pc can be used to modify this value.
     */
#       endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
    /**
     *  \image html Offset_pc.png
     *  The valid range for this property lies between -100% and +100%, whereas negative values
     *  will shifts the black level towards 0 and positive values to 255.
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyF offset_pc;
    /// \brief An enumerated integer property defining the pixel clock of the camera sensor in KHz.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraPixelClock.
     *
     *  \note
     *  Changing the pixel clock at runtime will have major impact on the overall behaviour of the
     *  camera. The pixel clock should therefore not be modified constantly during the execution
     *  of the application code. Whenever this property is changed, the next images caputured should
     *  be skipped if a device works with an active auto offset calibration controller circuit as then the
     *  control circuits on the camera might need one to three images until all control loops are
     *  locked again.
     *
     *  Every device or even different models belonging to the same device family may support different pixel clocks,
     *  thus it's crucial to query the valid clock modes for each sensor by reading the properties translation dictionary.
     */
    PropertyICameraPixelClock pixelClock_KHz;
    /// \brief An enumerated integer property defining the exposure mode the camera sensor is operated in.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExposeMode.
     *
     *  \note
     *  Not every device will offer the same options.
     *  Check for valid modes by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyICameraExposeMode::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyICameraExposeMode::getTranslationDictValue.
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyICameraExposeMode::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyICameraExposeMode exposeMode;
    /// \brief An integer property which defines the exposure time for an image in us.
    /**
     *  \image html Expose_us.png
     *
     */
    PropertyI expose_us;
    /// \brief An enumerated integer property defining the binning mode the camera is operated in.
    /**
     *  By default the no binning will be performed.
     *  \note
     *  Not every camera will support every binning mode. To find out which binning modes
     *  are supported by your camera you can query the properties translation table with
     *  the functions \b mvIMPACT::acquire::PropertyICameraBinningMode::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyICameraBinningMode::getTranslationDictValue
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyICameraBinningMode::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraBinningMode.
     */
    PropertyICameraBinningMode binningMode;
    ///  \brief An enumerated integer property defining the image transmission mode of the camera.
    /**
     *  If this property is set to \b mvIMPACT::acquire::ctmOff the 'normal' image
     *  captured by the cameras sensor will be transmitted. This is the default.
     *  To ensure correct operation the camera might define several test modes where
     *  some kind of well defined pattern will be transmitted.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTestMode. No every
     *  test mode will be supported by every device.
     */
    PropertyICameraTestMode testMode;
    /// \brief An enumerated integer property which defines what kind of trigger mode shall be used for an image acquisition.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTriggerMode.
     *
     *  \note
     *  Not every device will support every trigger mode. To find out which trigger modes are supported by a specific
     *  device at runtime the properties translation dictionary should be queried.
     *
     *  \note
     *  Also the sensor specific part of this manual will contain a list of trigger modes that are available for
     *  this specific image sensor.
     */
    PropertyICameraTriggerMode triggerMode;
    /// \brief An integer property defining the delay in us between an external trigger event an the begin of the sensor exposure.
    /**
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyI frameDelay_us;
    /// \brief Provides access to the control parameters for AGC and AEC.
    /**
     *  \note
     *  These settings will only affect the image if \b mvIMPACT::acquire::CameraSettingsBlueDevice::autoGainControl,
     *  \b mvIMPACT::acquire::CameraSettingsBlueDevice::autoExposeControl or both are set to active.
     */
    AutoControlParameters autoControlParameters;
    PYTHON_ONLY( %mutable; )
    /// \brief Provides access to the control parameters for AGC and AEC.
    /**
     *  \note
     *  These settings will only affect the image if \b mvIMPACT::acquire::CameraSettingsBlueDevice::autoGainControl,
     *  \b mvIMPACT::acquire::CameraSettingsBlueDevice::autoExposeControl or both are set to active.
     */
    AutoControlParameters& getAutoControlParameters( void )
    {
        return autoControlParameters;
    }
#       ifdef DOTNET_ONLY_CODE
    PropertyIAutoControlMode getAutoControlMode( void ) const
    {
        return autoControlMode;
    }
    PropertyIAutoGainControl getAutoGainControl( void ) const
    {
        return autoGainControl;
    }
    PropertyIAutoExposureControl getAutoExposeControl( void ) const
    {
        return autoExposeControl;
    }
    PropertyF getGain_dB( void ) const
    {
        return gain_dB;
    }
    PropertyF getOffset_pc( void ) const
    {
        return offset_pc;
    }
    PropertyICameraPixelClock getPixelClock_KHz( void ) const
    {
        return pixelClock_KHz;
    }
    PropertyICameraExposeMode getExposeMode( void ) const
    {
        return exposeMode;
    }
    PropertyI getExpose_us( void ) const
    {
        return expose_us;
    }
    PropertyICameraBinningMode getBinningMode( void ) const
    {
        return binningMode;
    }
    PropertyICameraTestMode getTestMode( void ) const
    {
        return testMode;
    }
    PropertyICameraTriggerMode getTriggerMode( void ) const
    {
        return triggerMode;
    }
    PropertyI getFrameDelay_us( void ) const
    {
        return frameDelay_us;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)

#   if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)
//-----------------------------------------------------------------------------
/// \brief A class to configure a HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) knee point (\b Device specific interface layout only).
/**
 *  \note
 *  Not every device will offer the features provided in this class. Before accessing properties call
 *  the function \b mvIMPACT::acquire::Component::isValid. Right now only the mvBlueFOX-x00wx types
 *  will support the features defined by this class.
 *
 *  \note
 *  Instances of these class can't be constructed directly. Its parameters can
 *  be accessed via an instance of the class \b mvIMPACT::acquire::CameraSettingsBlueFOX or
 *  \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class HDRKneePoint : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class HDRControl;
    explicit HDRKneePoint( HOBJ hObj ): ComponentCollection( hObj )
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( HDRControlVoltage_mV, "HDRControlVoltage_mV" );
        locator.bindComponent( HDRExposure_ppm, "HDRExposure_ppm" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property that can be used to define the control voltage in mV for this knee point.
    PropertyI HDRControlVoltage_mV;
    /// \brief An enumerated integer property that can be used to define the exposure time in ppm of the overall exposure time for this knee point.
    PropertyI HDRExposure_ppm;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getHDRControlVoltage_mV( void ) const
    {
        return HDRControlVoltage_mV;
    }
    PropertyI getHDRExposure_ppm( void ) const
    {
        return HDRExposure_ppm;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to control the HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) parameters of a device (\b Device specific interface layout only).
/**
 *  \note
 *  Not every device will offer the features provided in this class. Before accessing properties call
 *  the function \b mvIMPACT::acquire::Component::isValid. Right now only the mvBlueFOX-x00wx and
 *  mvBlueCOUGAR-x00wx types will support the features defined by this class.
 *
 *  \note
 *  Instances of these class can't be constructed directly. Its parameters can
 *  be accessed via an instance of the class \b mvIMPACT::acquire::CameraSettingsBlueFOX or
 *  \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class HDRControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
    friend class CameraSettingsBlueCOUGAR;
    friend class CameraSettingsBlueFOX;
    typedef std::vector<HDRKneePoint*> HDRKneePointContainer;
#       ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        bool                             m_boAvailable;
        HDRKneePointContainer            m_KneePoints;
        HOBJ                             m_HDRControlRoot;
        unsigned int                     m_refCnt;
        ReferenceCountedData() : m_boAvailable( false ), m_KneePoints(), m_HDRControlRoot( INVALID_ID ), m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            const HDRKneePointContainer::size_type HDRKneePointCnt = m_KneePoints.size();
            for( HDRKneePointContainer::size_type h = 0; h < HDRKneePointCnt; h++ )
            {
                delete m_KneePoints[h];
            }
        }
    }* m_pRefData;
#       endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    void bindPublicProperties( void )
    //-----------------------------------------------------------------------------
    {
        if( m_pRefData->m_HDRControlRoot != INVALID_ID )
        {
            ComponentLocator locator( m_pRefData->m_HDRControlRoot );
            locator.bindComponent( HDREnable, "HDREnable" );
            locator.bindComponent( HDRMode, "HDRMode" );
            locator.bindComponent( HDRKneePointCount, "HDRKneePointCount" );
        }
    }
    //-----------------------------------------------------------------------------
    explicit HDRControl( HOBJ hObj ): ComponentCollection( hObj ), m_pRefData( new ReferenceCountedData() )
        //-----------------------------------------------------------------------------
    {
        ComponentLocator locator( m_hRoot );
        HOBJ hRoot = locator.findComponent( "HDRControl" );
        if( hRoot != INVALID_ID )
        {
            m_hRoot = hRoot;
            m_pRefData->m_HDRControlRoot = m_hRoot;
            bindPublicProperties();
            locator.bindSearchBase( locator.searchbase_id(), "HDRControl" );
            // create the HDR knee point objects for this setting
            ComponentIterator it;
            locator.bindComponent( it, "HDRKneePoints" );
            if( it.isValid() )
            {
                it = it.firstChild();
                while( it.isValid() )
                {
                    m_pRefData->m_KneePoints.push_back( new HDRKneePoint( it.hObj() ) );
                    ++it;
                }
            }
            m_pRefData->m_boAvailable = true;
        }
    }
public:
    /// \brief Copy constructor
    /**
     *  Creates a new object from an existing device object. Keep in mind that this new object
     *  will provide access to the very same hardware and therefore you might as well use the original
     *  reference. This constructor is only provided for internal reference counting to guarantee correct operation of the
     *  objects of this class under all platforms and languages.
     */
    explicit HDRControl( const HDRControl& src ) : ComponentCollection( src ), m_pRefData( src.m_pRefData ), HDREnable( src.HDREnable ),
        HDRMode( src.HDRMode ), HDRKneePointCount( src.HDRKneePointCount )
    {
        ++( m_pRefData->m_refCnt );
    }
    ~HDRControl()
    {
        dealloc();
    }
#       ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::HDRControl objects
    HDRControl& operator=( const HDRControl& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++( m_pRefData->m_refCnt );
            bindPublicProperties();
        }
        return *this;
    }
#       endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief This function should be called to check if this device offers auto control parameters.
    /**
     *  \return
     *  - true if the device offers the features defined by this object.
     *  - false otherwise
     */
    bool isAvailable( void ) const
    {
        return m_pRefData->m_boAvailable;
    }
    /// \brief Returns a reference to a set of user definable parameters to configure a HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) knee point.
    /**
     *  Use the property \b mvIMPACT::acquire::HDRControl::HDRKneePointCount to find out how many knee points are available
     *  or to change the number of knee points.
     *
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr is invalid(too large) a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \return A reference to a set of user definable parameters to configure a HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) knee point.
     */
    HDRKneePoint& getHDRKneePoint(
        /// [in] The number of the knee point to be returned
        unsigned int nr ) const
    {
        return *( m_pRefData->m_KneePoints.at( nr ) );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property which can be used to enable/disable HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) mode.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean HDREnable;
    /// \brief An enumerated integer property which can be used to configure the HDR (<b>H</b>igh <b>D</b>ynamic <b>R</b>ange) mode to work with.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraHDRMode.
     */
    PropertyICameraHDRMode HDRMode;
    /// \brief An integer property to define the number of knee points to work with when \b mvIMPACT::acquire::HDRControl::HDRMode is set to \b mvIMPACT::acquire::cHDRmUser.
    PropertyI HDRKneePointCount;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIBoolean getHDREnable( void ) const
    {
        return HDREnable;
    }
    PropertyICameraHDRMode getHDRMode( void ) const
    {
        return HDRMode;
    }
    PropertyI getHDRKneePointCount( void ) const
    {
        return HDRKneePointCount;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #if !defined(IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION) || !defined(IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION)

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief \b mvBlueFOX related camera settings(\b Device specific interface layout only).
/**
 *  This class contains properties to control the way the image sensor of the
 *  \b mvBlueFOX behaves.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsBlueFOX : public CameraSettingsBlueDevice
//-----------------------------------------------------------------------------
{
    HDRControl HDRControl_;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsBlueFOX object
    explicit CameraSettingsBlueFOX(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBlueDevice( pDev, settingName ), HDRControl_( m_hRoot ),
        offsetAutoCalibration(), offsetCorrection_pc(), offsetAutoBlackLevel(),
        offsetAutoBlackSpeed(), lineDelay_clk(), advancedOptions(),
        flashToExposeDelay_us(), flashMode(), flashType(), shutterMode(),
        triggerSource(), sensorTimingMode()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( offsetAutoCalibration, "OffsetAutoCalibration" );
        locator.bindComponent( offsetCorrection_pc, "OffsetCorrection_pc" );
        locator.bindComponent( offsetAutoBlackLevel, "OffsetAutoBlackLevel" );
        locator.bindComponent( offsetAutoBlackSpeed, "OffsetAutoBlackSpeed" );
        locator.bindComponent( lineDelay_clk, "LineDelay_clk" );
        locator.bindComponent( advancedOptions, "AdvancedOptions" );
        locator.bindComponent( flashToExposeDelay_us, "FlashToExposeDelay_us" );
        locator.bindComponent( flashMode, "FlashMode" );
        locator.bindComponent( flashType, "FlashType" );
        locator.bindComponent( shutterMode, "ShutterMode" );
        locator.bindComponent( triggerSource, "TriggerSource" );
        locator.bindComponent( sensorTimingMode, "SensorTimingMode" );
    }
    /// \brief Returns the \b mvIMPACT::acquire::HDRControl object associated with this setting.
    HDRControl& getHDRControl( void )
    {
        return HDRControl_;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the offset calibration mode.
    /**
     *  If this property is set to \b mvIMPACT::acquire::aocOff
     *  the offset can be adjusted manually by modifying the \b mvIMPACT::acquire::CameraSettingsBlueFOX::offset_pc and
     *  mvIMPACT::acquire::CameraSettingsBlueFOX::offsetCorrection_pc properties.
     *
     *  If set to \b mvIMPACT::acquire::aocOn the offset is approx. 10. That means,
     *  that the medium pixel count is 10, in case no light is falling on the sensor.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAutoOffsetCalibration.
     */
    PropertyIAutoOffsetCalibration offsetAutoCalibration;
    /// \brief A float property defining and \b additional analogue sensor offset in percent of the allowed range (sensor specific) \b per \b color \b channel.
    /**
     *  Together with value of the property \b mvIMPACT::acquire::CameraSettingsBlueDevice::offset_pc this property influences the average grey level
     *  when no light reaches the sensor.
     *
     *  This property stores 4 values.
     *  - The value at index 0 applies to the green component of the red-green rows of the bayer pattern
     *  - The value at index 1 applies to the red component of the bayer pattern
     *  - The value at index 2 applies to the green component of the blue-green rows of the bayer pattern
     *  - The value at index 3 applies to the blue component of the bayer pattern
     *
     *  \note
     *  This property might not be supported by every device. Therefore always call the function
     *  \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  In addition to that this property must be enabled explicitly by setting the advanced option bit
     *  \b mvIMPACT::acquire::daoEnablePerChannelOffsetCorrection of the property
     *  \b mvIMPACT::acquire::CameraSettingsBlueFOX::advancedOptions.
     *  \sa
     *  \b mvIMPACT::acquire::CameraSettingsBlueDevice::offset_pc
     */
    PropertyF offsetCorrection_pc;
    /// \brief An integer property defining the digital value assumed as black.
    /**
     *  Modifying this property in rare cases might be useful to achieve a higher dynamic range for noise close to the black
     *  level.
     *
     *  \note
     *  As the \b mvBlueFOX internally works with a 10 bit analog to digital conversion increasing this
     *  value by 4 will result in a black level increased by about 1 in the resulting 8 bit raw image
     *  delivered from the camera.
     */
    PropertyI offsetAutoBlackLevel;
    /// \brief An enumerated integer property defining the speed of the auto offset calibration circuit.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBlueFOXOffsetAutoBlackSpeed.
     */
    PropertyIBlueFOXOffsetAutoBlackSpeed offsetAutoBlackSpeed;
    /// \brief An integer property defining the delay in clocks between two lines.
    /**
     *  Can be used to slow down the image readout the reduce the peak bandwidth required
     *  for the transfer.
     */
    PropertyI lineDelay_clk;
    /// \brief An enumerated integer property defining a collection of advanced options that can be enabled or disabled.
    /**
     *  This property is highly sensor dependent. Each sensor might offer a different sub set of the options offered.
     *  Only experienced users should work with this feature.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceAdvancedOptions.
     */
    PropertyIDeviceAdvancedOptions advancedOptions;
    /// \brief An integer property defining the delay in us betwenn the start of the flash signal output and the beginning of the expose period of the image sensor.
    /**
     *  \note
     *  This property is mainly provided for compatibility reasons. Previous versions of the
     *  \b mvBlueFOX driver produced a fixed delay of about 300 us between the beginning of the
     *  flash signal creation and the beginning of the sensors expose period. Newer versions of the
     *  driver do \b NOT produce this delay anymore which might cause a different illumination
     *  for applications using very intensive, short flash and expose times. Applications relying on
     *  the delay introduced in older driver versions should set this property to
     *  '300' by calling \b mvIMPACT::acquire::PropertyI::write.
     *
     *  \sa
     *  \b mvIMPACT::acquire::CameraSettingsBlueDevice::expose_us
     */
    PropertyI flashToExposeDelay_us;
    /// \brief An enumerated integer property defining the behaviour of the flash output of the camera (if available).
    /**
     *  If this property is set the cameras flash output (if available) will be pulsed
     *  during the exposure period of the sensor. In that case the pulse width will be equal
     *  to the exposure time.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraFlashMode.
     *
     *  \sa
     *  \b mvIMPACT::acquire::CameraSettingsBlueFOX::flashToExposeDelay_us
     */
    PropertyICameraFlashMode flashMode;
    /// \brief An enumerated integer property defining the type of the flash output of the camera (if available).
    /**
     *  Depending of the camera sensor type (\a rolling shutter or \a frame \a shutter
     *  there well be different modes available
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraFlashType.
     */
    PropertyICameraFlashType flashType;
    /// \brief An enumerated integer property defining the shutter mode to use.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraShutterMode.
     */
    PropertyICameraShutterMode shutterMode;
    /// \brief An enumerated integer property defining where the trigger signal is expected to occur.
    /**
     *  When working with hardware generated signals via \b mvIMPACT::acquire::RTCtrProgram objects
     *  this property should be set to \b mvIMPACT::acquire::ctsRTCtrl as otherwise
     *  the program will not affect the behaviour of the camera.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTriggerSource.
     */
    PropertyICameraTriggerSource triggerSource;
    /// \brief An enumerated integer property defining the current image sensor timing mode.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBlueFOXSensorTiming.
     */
    PropertyIBlueFOXSensorTiming sensorTimingMode;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIAutoOffsetCalibration getOffsetAutoCalibration( void ) const
    {
        return offsetAutoCalibration;
    }
    PropertyF getOffsetCorrection_pc( void ) const
    {
        return offsetCorrection_pc;
    }
    PropertyI getOffsetAutoBlackLevel( void ) const
    {
        return offsetAutoBlackLevel;
    }
    PropertyIBlueFOXOffsetAutoBlackSpeed getOffsetAutoBlackSpeed( void ) const
    {
        return offsetAutoBlackSpeed;
    }
    PropertyI getLineDelay_clk( void ) const
    {
        return lineDelay_clk;
    }
    PropertyIDeviceAdvancedOptions getAdvancedOptions( void ) const
    {
        return advancedOptions;
    }
    PropertyI getFlashToExposeDelay_us( void ) const
    {
        return flashToExposeDelay_us;
    }
    PropertyICameraFlashMode getFlashMode( void ) const
    {
        return flashMode;
    }
    PropertyICameraFlashType getFlashType( void ) const
    {
        return flashType;
    }
    PropertyICameraShutterMode getShutterMode( void ) const
    {
        return shutterMode;
    }
    PropertyICameraTriggerSource getTriggerSource( void ) const
    {
        return triggerSource;
    }
    PropertyIBlueFOXSensorTiming getSensorTimingMode( void ) const
    {
        return sensorTimingMode;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief \b mvBlueCOUGAR and \b mvBlueLYNX-M7 related camera settings (\b Device specific interface layout only).
/**
 *  This class contains properties to control the way the image sensor(s) of the
 *  \b mvBlueCOUGAR or \b mvBlueLYNX-M7 behaves.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsBlueCOUGAR : public CameraSettingsBlueDevice
//-----------------------------------------------------------------------------
{
    typedef std::vector<TriggerControl*> TriggerControlVector;
    TriggerControlVector m_triggerControls;
    HDRControl HDRControl_;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR object
    explicit CameraSettingsBlueCOUGAR(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBlueDevice( pDev, settingName ),
        m_triggerControls(), HDRControl_( m_hRoot ),
        triggerInterface(), triggerSource()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( frameRate_Hz, "FrameRate_Hz" );
        locator.bindComponent( partialScanMode, "PartialScanMode" );
        locator.bindComponent( triggerInterface, "TriggerInterface" );
        locator.bindComponent( triggerSource, "TriggerSource" );
        if( locator.findComponent( "TriggerControls" ) != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "TriggerControls" );
            ComponentIterator it( locator.searchbase_id() );
            it = it.firstChild();
            while( it.isValid() )
            {
                m_triggerControls.push_back( new TriggerControl( it ) );
                ++it;
            }
        }
    }
    virtual ~CameraSettingsBlueCOUGAR()
    {
        TriggerControlVector::size_type vSize = m_triggerControls.size();
        for( TriggerControlVector::size_type i = 0; i < vSize; i++ )
        {
            delete m_triggerControls[i];
        }
    }
    /// \brief Returns the number of \b mvIMPACT::acquire::TriggerControls available for this device.
    /**
     *  This might be 0 if the device either does not support this feature.
     */
    unsigned int getTriggerControlCount( void ) const
    {
        return static_cast<unsigned int>( m_triggerControls.size() );
    }
    /// \brief Returns a const pointer to a \b mvIMPACT::acquire::TriggerControl object.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     *
     *  \note
     *  The features of each \b mvIMPACT::acquire::TriggerControl will only be visibile and thus available,
     *  when \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR::triggerInterface
     *  is set to \b mvIMPACT::acquire::dtiAdvanced.
     */
    const TriggerControl* triggerControl(
        /// [in] The number of the trigger control
        unsigned int nr ) const
    {
        return m_triggerControls.at( nr );
    }
    /// \brief Returns a const pointer to a \b mvIMPACT::acquire::TriggerControl object or NULL if the name passed to the function does not specify a valid trigger control.
    const TriggerControl* triggerControl(
        /// [in] The name of the trigger control
        const std::string& name ) const
    {
        const TriggerControlVector::size_type cnt = m_triggerControls.size();
        for( TriggerControlVector::size_type i = 0; i < cnt; i++ )
        {
            const TriggerControl* p = triggerControl( static_cast<unsigned int>( i ) );
            if( p->getDescription() == name )
            {
                return p;
            }
        }
        return 0;
    }
    /// \brief Returns the \b mvIMPACT::acquire::HDRControl object associated with this setting.
    HDRControl& getHDRControl( void )
    {
        return HDRControl_;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A float property defining the frame rate (in Hz) this device shall use to transfer images.
    /**
     *  Depending on other parameters the desired frame rate might not be achievable (e.g. if the exposure time
     *  is higher than the frame period. In that case the device will automatically reduce the frame rate.
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyF frameRate_Hz;
    /// \brief An enumerated integer property defining whether partial scan mode shall be used or not.
    /**
     *  This property currently is only available for mvBlueCOUGAR-S devices. When partial scan mode is active, the
     *  frame rate can no longer be controlled be the property \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR::frameRate_Hz.
     *  The device will always transmit frames as fast as possible instead.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean partialScanMode;
    /// \brief An enumerated integer property defining which view of the trigger interface to use.
    /**
     *  This property mainly switches the visibility of trigger the different trigger interfaces. After applying changes to
     *  this property all trigger related features might have a different visibility. However the documentation will state
     *  what features are available in which mode.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceTriggerInterface.
     */
    PropertyIDeviceTriggerInterface triggerInterface;
    /// \brief An enumerated integer property defining where the trigger signal is expected to occur.
    /**
     *  When working with hardware generated signals via \b mvIMPACT::acquire::RTCtrProgram objects
     *  this property should be set to \b mvIMPACT::acquire::ctsRTCtrl as otherwise
     *  the program will not affect the behaviour of the camera.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTriggerSource.
     *
     *  \note
     *  This feature will only be visibile and thus available, when \b mvIMPACT::acquire::CameraSettingsBlueCOUGAR::triggerInterface
     *  is set to \b mvIMPACT::acquire::dtiStandard.
     */
    PropertyICameraTriggerSource triggerSource;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyF getFrameRate_Hz( void ) const
    {
        return frameRate_Hz;
    }
    PropertyIBoolean getPartialScanMode( void ) const
    {
        return partialScanMode;
    }
    PropertyIDeviceTriggerInterface getTriggerInterface( void ) const
    {
        return triggerInterface;
    }
    PropertyICameraTriggerSource getTriggerSource( void ) const
    {
        return triggerSource;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class to access frame grabber related camera settings(\b Device specific interface layout only).
/**
 *  This class contains properties to control the way the a frame grabber
 *  behaves in connection with the connected camera.
 *
 *  \note
 *  Not every frame grabber will support every property defined in this class. Please
 *  refer to the description of the individual properties and make heavy use of the
 *  function \b mvIMPACT::acquire::Component::isValid in order to check whether
 *  a property is available for the device.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraSettingsFrameGrabber : public CameraSettingsBase
//-----------------------------------------------------------------------------
{
    typedef std::vector<TriggerControl*> TriggerControlVector;
    TriggerControlVector m_triggerControls;
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraSettingsFrameGrabber object
    explicit CameraSettingsFrameGrabber(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : CameraSettingsBase( pDev, settingName ),
        type(), aoiMode(), gain_dB(), offset_mV(), scanClock(), triggerMode(), softwareTriggerPeriod_ms(),
        serialPortBaudrate(), interlacedMode(), acquisitionField(), scanRateMode(), scanRate_kHz(),
        luminance_pc(), saturation_pc(), hue_pc(), contrast_pc(), lineCounter()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( type, "Type" );
        locator.bindComponent( aoiMode, "AoiMode" );
        locator.bindComponent( gain_dB, "Gain_dB" );
        locator.bindComponent( offset_mV, "Offset_mV" );
        locator.bindComponent( scanClock, "ScanClock" );
        locator.bindComponent( triggerMode, "TriggerMode", 0, 0 );
        locator.bindComponent( softwareTriggerPeriod_ms, "SoftwareTriggerPeriod_ms" );
        locator.bindComponent( serialPortBaudrate, "SerialPortBaudRate" );
        locator.bindComponent( interlacedMode, "InterlacedMode" );
        locator.bindComponent( acquisitionField, "AcquisitionField" );
        locator.bindComponent( scanRateMode, "ScanRateMode" );
        locator.bindComponent( scanRate_kHz, "ScanRate_kHz" );
        locator.bindComponent( luminance_pc, "Luminance_pc" );
        locator.bindComponent( saturation_pc, "Saturation_pc" );
        locator.bindComponent( hue_pc, "Hue_pc" );
        locator.bindComponent( contrast_pc, "Contrast_pc" );
        locator.bindComponent( lineCounter, "LineCounter" );
        if( locator.findComponent( "TriggerControls" ) != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "TriggerControls" );
            ComponentIterator it( locator.searchbase_id() );
            if( it.isValid() )
            {
                it = it.firstChild();
                while( it.isValid() )
                {
                    m_triggerControls.push_back( new TriggerControl( it ) );
                    ++it;
                }
            }
        }
    }
    virtual ~CameraSettingsFrameGrabber()
    {
        TriggerControlVector::size_type vSize = m_triggerControls.size();
        for( TriggerControlVector::size_type i = 0; i < vSize; i++ )
        {
            delete m_triggerControls[i];
        }
    }
    /// \brief Returns the number of \b mvIMPACT::acquire::TriggerControls available for this device.
    /**
     *  This might be 0 if the device either does not support this feature.
     */
    unsigned int getTriggerControlCount( void ) const
    {
        return static_cast<unsigned int>( m_triggerControls.size() );
    }
    /// \brief Returns a const pointer to a \b mvIMPACT::acquire::TriggerControl object.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If \a nr references an invalid pin a STL out_of_range exception
     *  will be thrown.
     *  \endif
     */
    const TriggerControl* triggerControl(
        /// [in] The number of the trigger control
        unsigned int nr ) const
    {
        return m_triggerControls.at( nr );
    }
    /// \brief Returns a const pointer to a \b mvIMPACT::acquire::TriggerControl object or NULL if the name passed to the function does not specify a valid trigger control.
    const TriggerControl* triggerControl(
        /// [in] The name of the trigger control
        const std::string& name ) const
    {
        const TriggerControlVector::size_type cnt = m_triggerControls.size();
        for( TriggerControlVector::size_type i = 0; i < cnt; i++ )
        {
            const TriggerControl* p = triggerControl( static_cast<unsigned int>( i ) );
            if( p->getDescription() == name )
            {
                return p;
            }
        }
        return 0;
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the camera description used for the image acquisition.
    /**
     *  This property \b ALWAYS defines a translation dictionary containing a string representation
     *  and a numerical value for the camera descriptions available for the device. The string
     *  representation of the camera description will be build from the property name of the class this
     *  camera description belongs to (e.g. \a 'Standard', \a 'NonStandard' or \a 'CameraLink') and
     *  the name of the camera description itself. Assuming a the device can handle standard
     *  video cameras and a description for a camera named \a 'MyCCIRCamera' will therefore
     *  add \a 'Standard_MyCCIRCamera' and a unique numerical representation to the properties
     *  translation dictionary.
     */
    PropertyI type;
    /// \brief An enumerated integer property defining the used AOI mode for the image capture.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraAoiMode.
     */
    PropertyICameraAoiMode aoiMode;
    /// \brief A float property defining the gain in dB.
    /**
     *  This is the gain applied during the analogue to digital conversion inside the frame
     *  grabber, not applied inside the camera.
     *
     *  * The formula for given \c gain_dB is
     *  \code
     *  gain_x= 10 ^ (gain_dB/20)
     *  \endcode
     *
     *  \b Example:
     *
     *  \code
     *  gain_x = 10 ^ (6/20) = 1.995
     *  \endcode
     *
     *  I.e. increasing the gain setting by 6dB corresponds to factor of 2.
     *
     *  The formula for given gain_x is
     *  \code
     *  gain_dB = 20*log(gain_x) dB
     *  \endcode
     *
     *  Where: \n
     *  \c gain_dB: MATRIX VISION gain value (logarithmic) \n
     *  \c gain_x: multiplicative gain \n
     *  \c ^: power function \n
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyF gain_dB;
    /// \brief An integer property defining the offset in milli-Volt.
    /**
     *  This is the analogue offset applied during the analogue to digital conversion inside the frame
     *  grabber, not applied inside the camera.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI offset_mV;
    /// \brief An enumerated integer property defining the scan clock mode used for the current acquisition.
    /**
     *  Valid modes are defined by the enumeration \b mvIMPACT::acquire::TScanClock.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception! Digital frame grabbers will however \b NOT support
     *  this property.
     */
    PropertyIScanClock scanClock;
    /// \brief An enumerated integer property defining the behaviour of the frame grabbers trigger input.
    /**
     *  \note
     *  All trigger modes are defined by \b mvIMPACT::acquire::TDeviceTriggerMode. However
     *  not every device will offer all these trigger modes but a subset of them. Valid trigger modes therefore
     *  can be found by reading the properties translation dictionary.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyIDeviceTriggerMode triggerMode;
    /// \brief An integer property defining the software simulated trigger period.
    /**
     *  When \b mvIMPACT::acquire::CameraSettingsFrameGrabber::triggerMode is set to
     *  \b mvIMPACT::acquire::dtmPeriodically this property defines the
     *  interval between two consecutive external trigger events simulated by the driver.
     *
     *  Currently this property is only available for \b mvTITAN and \b mvGAMMA frame grabbers.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI softwareTriggerPeriod_ms;
    /// \brief An enumerated integer property defining the baud rate of the serial communication port between frame grabber and camera.
    /**
     *  Currently this property is only available for CameraLink&reg; frame grabbers.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraSerialPortBaudRate.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyICameraSerialPortBaudRate serialPortBaudrate;
    /// \brief An enumerated integer property defining how the image from the camera shall be captured.
    /**
     *  If a video signal is transmitted to the capture device in an interlaced format, the device
     *  can either capture each individual frame, or merge two frames together into a complete image.
     *
     *  This property will be invisible when the camera description selected by the property
     *  \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type does define it property
     *  \b mvIMPACT::acquire::CameraDescriptionBase::interlacedType to be \b mvIMPACT::acquire::citNone
     *  as the decision whether an interlaced image shall be reconstructed from frames or left
     *  as individual frame is redundant if the video source transmitts full frames only.
     *
     *  The visibility of a property can be checked by calling \b mvIMPACT::acquire::Component::isVisible
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TInterlacedMode.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyIInterlacedMode interlacedMode;
    /// \brief An enumerated integer property defining which frame(s) of an interlaced video signal will be digitised or triggers the start of the acquisition.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAcquisitionField.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyIAcquisitionField acquisitionField;
    /// \brief An enumerated integer property defining the scan rate mode for this setting.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDeviceScanRateMode.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyIDeviceScanRateMode scanRateMode;
    /// \brief An integer property defining the current scan frequency of the capture device.
    /**
     *  This property is only visible if the \b mvIMPACT::acquire::CameraSettingsFrameGrabber::scanRateMode
     *  property is \b NOT in an auto mode.
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI scanRate_kHz;
    /// \brief An integer property defining the luminance value to be applied to the image data by the frame grabber in per cent.
    /**
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI luminance_pc;
    /// \brief An integer property defining the saturation value to be applied to the image data by the frame grabber in per cent.
    /**
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI saturation_pc;
    /// \brief An integer property defining the hue value to be applied to the image data by the frame grabber in per cent.
    /**
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI hue_pc;
    /// \brief An integer property defining the contrast value to be applied to the image data by the frame grabber in per cent.
    /**
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyI contrast_pc;
    /// \brief An enumerated integer property defining the way line counter information is handled by the device.
    /**
     *  This is interessting for line scan camera applications. More information can be found a the documentation
     *  of the enumeration \b mvIMPACT::acquire::TLineCounter.
     *
     *  \note
     *  This property will not be available for every frame grabber, so before using it, check
     *  if the property is available at all by calling \b mvIMPACT::acquire::Component::isValid. Accessing
     *  an invalid property will raise an exception!
     */
    PropertyILineCounter lineCounter;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getType( void ) const
    {
        return type;
    }
    PropertyICameraAoiMode getAoiMode( void ) const
    {
        return aoiMode;
    }
    PropertyF getGain_dB( void ) const
    {
        return gain_dB;
    }
    PropertyI getOffset_mV( void ) const
    {
        return offset_mV;
    }
    PropertyIScanClock getScanClock( void ) const
    {
        return scanClock;
    }
    PropertyIDeviceTriggerMode getTriggerMode( void ) const
    {
        return triggerMode;
    }
    PropertyI getSoftwareTriggerPeriod_ms( void ) const
    {
        return softwareTriggerPeriod_ms;
    }
    PropertyICameraSerialPortBaudRate getSerialPortBaudRate( void ) const
    {
        return serialPortBaudrate;
    }
    PropertyIInterlacedMode getInterlacedMode( void ) const
    {
        return interlacedMode;
    }
    PropertyIAcquisitionField getAcquisitionField( void ) const
    {
        return acquisitionField;
    }
    PropertyIDeviceScanRateMode getScanRateMode( void ) const
    {
        return scanRateMode;
    }
    PropertyI getScanRate_kHz( void ) const
    {
        return scanRate_kHz;
    }
    PropertyI getLuminance_pc( void ) const
    {
        return luminance_pc;
    }
    PropertyI getSaturation_pc( void ) const
    {
        return saturation_pc;
    }
    PropertyI getHue_pc( void ) const
    {
        return hue_pc;
    }
    PropertyI getContrast_pc( void ) const
    {
        return contrast_pc;
    }
    PropertyILineCounter getLineCounter( void ) const
    {
        return lineCounter;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class to describe a camera (\b Device specific interface layout only).
/**
 *  A camera description object provides an abstract way to prepare the capture
 *  device (e.g. a frame grabber) for the connected imaging device (e.g. a camera).
 *
 *  By selecting one of the available camera descriptions the underlying hardware
 *  will use the information provided in the camera description to prepare the
 *  image capture. Therefore it is crucial to select a camera description
 *  that matches the connected video signal as close as possible. If no description
 *  for the camera or imaging device connected is available a new description
 *  should be generated from one of the existing ones. This can be achieved by calling
 *  the function \b mvIMPACT::acquire::CameraDescriptionBase::copyDescription. A new
 *  camera description with the name that is passed to the function will be created.
 *
 *  \note
 *  The name must be unique. There can't be two descriptions with the same name
 *  belonging to the same generic class (i.e. \a 'Standard' or \a 'CameraLink'&reg;). However
 *  it's perfectly legal to have a camera description for standard analogue video signals
 *  and one for e.g. CameraLink&reg; signals with the same name.
 *
 *  This new description will be an exact copy of the one the copy function has been
 *  executed for. After creation this description can be selected e.g. by setting the
 *  property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type to the name of the
 *  new description.
 *
 *  \note
 *  Please note that the name passed to the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type
 *  does \b NOT exactly correspond to the name assigned to the new camera description.
 *  It is a combination of the class the camera is belonging to (e.g. \a 'Standard') and the actual name.
 *  So to select a camera description the name must be build from teh return value of a call to
 *  \b mvIMPACT::acquire::CameraDescriptionBase::getClassName, an underscore ('_') and the
 *  actual name of the description: &lt;class name&gt;_&lt;desc. name&gt;
 *
 *  Example: For a standard description with the name \a 'MyCam' the full name would be \a 'Standard_MyCam'.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *  #include <iostream>
 *
 *  using namespace std;
 *
 *  //-----------------------------------------------------------------------------
 *  int main( int argc, char* argv[] )
 *  //-----------------------------------------------------------------------------
 *  {
 *    mvIMPACT::acquire::DeviceManager devMgr;
 *    mvIMPACT::acquire::Device* pDev = devMgr[0];
 *    if( !pDev )
 *    {
 *      cout << "No device found" << endl;
 *      return 0;
 *    }
 *
 *    try
 *    {
 *      // the next line will raise an exception if this device does not
 *      // support camera descriptions (e.g. if it is a USB camera rather than a frame grabber)
 *      mvIMPACT::acquire::CameraDescriptionManager cdm(pDev);
 *      mvIMPACT::acquire::CameraDescriptionStandard* pCam = cdm.cameraDescriptionStandard( "Generic" );
 *      pCam->copyDescription( "MyCam" );
 *      // Get access to the newly created description
 *      mvIMPACT::acquire::CameraDescriptionStandard* pMyCam = cdm.cameraDescriptionStandard( "MyCam" );
 *      // Modify something
 *      pMyCam->videoStandard.write( vsRS170 );
 *      pMyCam->scanStandard.write( ssITU601 );
 *      // now save the description to make it available in all future sessions
 *      pMyCam->exportDescription();
 *      // select the newly created description
 *      mvIMPACT::acquire::CameraSettingsFrameGrabber cs(pDev);
 *      // the name for this property is always a combination of the class the camera
 *      // description is belonging to and the actual name:
 *      cs.type.writeS( pMyCam->getClassName() + "_" + pMyCam->name.read() );
 *    }
 *    catch( const ImpactAcquireException& e )
 *    {
 *      cout << "An exception occurred: " << e.getErrorString() << endl;
 *    }
 *    return 0;
 *  }
 * \endcode
 *  \endif
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionBase : public ComponentCollection
//-----------------------------------------------------------------------------
{
    HDRV m_hDrv;
protected:
    explicit CameraDescriptionBase( HDRV hDrv, HLIST hList ) : ComponentCollection( hList ), m_hDrv( hDrv ), name(),
        videoOutput(), interlacedType(), aoiHeight(), aoiStartX(), aoiStartY(), aoiWidth()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( name, "Name" );
        locator.bindComponent( videoOutput, "VideoOutput" );
        locator.bindComponent( interlacedType, "InterlacedType" );
        locator.bindSearchBase( locator.searchbase_id(), "ActiveVideoAoi" );
        locator.bindComponent( aoiHeight, "H" );
        locator.bindComponent( aoiStartX, "X" );
        locator.bindComponent( aoiStartY, "Y" );
        locator.bindComponent( aoiWidth, "W" );
    }
public:
    /// \brief Returns the class this description is belonging to.
    /**
     *  \return The class this description is belonging to.
     */
    std::string getClassName( void ) const
    {
        return ComponentList( m_hRoot ).contentDescriptor();
    }
    /// \brief Creates a new camera description list as a copy of the current one.
    /**
     *  This function can be used to create a new camera description list. It will create
     *  a deep copy of the properties referenced by this camera description and will append the new description
     *  to the list of camera descriptions.
     *
     *  \note
     *  This will \b NOT store the new description permanently. After the driver has been
     *  closed, this data will be lost. To store the new list in a way that it will be available
     *  the next time the driver/device is opened again, \b mvIMPACT::acquire::CameraDescriptionBase::exportDescription must
     *  be called with the handle of the newly created description after this function.
     *  \sa
     *  \b mvIMPACT::acquire::CameraDescriptionBase::exportDescription, \n
     *  \b mvIMPACT::acquire::CameraDescriptionBase::importDescription
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - \b mvIMPACT::acquire::DEV_INPUT_PARAM_INVALID if new name matches a description that is already
     *  existing.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int copyDescription(
        /// [in] The name for the newly created camera description. This name
        /// \b MUST NOT match a description already existing.
        const std::string& newName ) const
    {
        return DMR_CopyCameraDescription( m_hDrv, m_hRoot, newName.c_str() );
    }
    /// \brief Stores the current camera description on disc.
    /**
     *  This function can be used to store the current settings of a camera description
     *  permanently so that the next time the driver is initialised these settings are restored.
     *
     *  When exporting a camera description a file in XML format will be written to disc. Under
     *  Windows&reg; camera descriptions will be stored under
     *  Windows&reg; camera descriptions will be stored under
     * \code
     *  %ALLUSERS%\Documents\MATRIX VISION\mvIMPACT acquire\CameraFiles
     * \endcode
     *  or
     * \code
     *  %MVIMPACT_ACQUIRE_DATA_DIR%\CameraFiles
     * \endcode
     *  which will point to the same folder), under Linux&reg; this directory will be
     *  \code
     *  /etc/matrix-vision/mvimpact-acquire/camerafiles
     *  \endcode
     *  while under other platforms these files will end up in the current working directory. This behaviour can be modified
     *  by writing the the property \b mvIMPACT::acquire::Device::customDataDirectory before initialising the
     *  device.
     *
     *  \sa
     *  \b mvIMPACT::acquire::CameraDescriptionBase::copyDescription, \n
     *  \b mvIMPACT::acquire::CameraDescriptionBase::importDescription
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int exportDescription( void ) const
    {
        return DMR_ExportCameraDescription( m_hDrv, m_hRoot );
    }
    /// \brief Updates a camera description with the data stored in a previous session or with the original default data.
    /**
     *  When importing a camera description the device driver will try to retrieve the parameters
     *  needed from a XML file. Under Windows&reg; camera descriptions MUST be stored under
     * \code
     *  %ALLUSERS%\Documents\MATRIX VISION\mvIMPACT acquire\CameraFiles
     * \endcode
     *  or
     * \code
     *  %MVIMPACT_ACQUIRE_DATA_DIR%\CameraFiles
     * \endcode
     *  which will point to the same folder), under Linux&reg; this directory will be
     *  \code
     *  /etc/matrix-vision/mvimpact-acquire/camerafiles
     *  \endcode
     *  while under other platforms these files MUST be located in the current working directory.
     *  This behaviour can be modified by writing the the property \b mvIMPACT::acquire::Device::customDataDirectory before initialising the device.
     *  To get access to a XML description file within the application, these file must be copied to
     *  this directory \b BEFORE the device is initialised. During the init process the
     *  device driver will process every file located under this location and will add them to
     *  the internal list of descriptions. Every camera located during this process and also descriptions
     *  created later on during the program operation can be selected via the property
     *  \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type.
     *
     *  \note
     *  In order not to bloat the device driver with redundant data it's wise only to store the
     *  descripitions needed for the application to run under this location.
     *
     *  \note
     *  Only camera descripitions supported by the capture device will be added to the device
     *  drivers internal list and will therefore be accessible via the
     *  \b mvIMPACT::acquire::CameraDescriptionManager.
     *
     *  To restore the default values valid during the driver was initialised the function
     *  \b mvIMPACT::acquire::ComponentCollection::restoreDefault can be used as well.
     *
     *  \sa
     *  \b mvIMPACT::acquire::CameraDescriptionBase::exportDescription, \n
     *     mvIMPACT::acquire::CameraDescriptionBase::copyDescription
     *  \return
     *  - \b mvIMPACT::acquire::DMR_NO_ERROR if successful.
     *  - A negative error code of type \b mvIMPACT::acquire::TDMR_ERROR or \b mvIMPACT::acquire::TPROPHANDLING_ERROR otherwise.
     */
    int importDescription( void ) const
    {
        return DMR_ImportCameraDescription( m_hDrv, m_hRoot );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A string property \b read-only containing the name of this camera description list.
    PropertyS name;
    /// \brief An enumerated integer property defining the type of video outputs this camera offers.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraOutput.
     *
     *  \note
     *  This property will be \b (read-only) for camera descriptions of type
     *  \b mvIMPACT::acquire::CameraDescriptionCameraLink and \b mvIMPACT::acquire::CameraDescriptionSDI
     *  as for these camera types the video output depends on certain other parameters according to the
     *  corresponding standard.
     */
    PropertyICameraOutput videoOutput;
    /// \brief An enumerated integer property defining how the camera transmits image data.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraInterlacedType.
     */
    PropertyICameraInterlacedType interlacedType;
    /// \brief An integer property defining the number of active lines to capture from the camera.
    PropertyI aoiHeight;
    /// \brief An integer property defining the X-offset in pixels to the first active pixel to capture.
    PropertyI aoiStartX;
    /// \brief An integer property defining the Y-offset in lines to the first active line to capture.
    PropertyI aoiStartY;
    /// \brief An integer property defining the number of active pixels to capture per line.
    PropertyI aoiWidth;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyS getName( void ) const
    {
        return name;
    }
    PropertyICameraOutput getVideoOutput( void ) const
    {
        return videoOutput;
    }
    PropertyICameraInterlacedType getInterlacedType( void ) const
    {
        return interlacedType;
    }
    PropertyI getAoiHeight( void ) const
    {
        return aoiHeight;
    }
    PropertyI getAoiStartX( void ) const
    {
        return aoiStartX;
    }
    PropertyI getAoiStartY( void ) const
    {
        return aoiStartY;
    }
    PropertyI getAoiWidth( void ) const
    {
        return aoiWidth;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class to describe an arbitrary digital camera signal(\b Device specific interface layout only).
/**
 *  This class provides access to all settings specific for digital video signals.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionDigitalBase : public CameraDescriptionBase
//-----------------------------------------------------------------------------
{
protected:
    explicit CameraDescriptionDigitalBase( HDRV hDrv, HLIST hList ) : CameraDescriptionBase( hDrv, hList ), bitsPerPixel(),
        format(), bayerParity()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( bitsPerPixel, "BitsPerPixel" );
        locator.bindComponent( format, "Format" );
        locator.bindComponent( bayerParity, "BayerParity" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the number of bits per pixel currently transmitted by this camera.
    PropertyI bitsPerPixel;
    /// \brief An enumerated integer property defining the data format the camera is sending image data.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraDataFormat.
     */
    PropertyICameraDataFormat format;
    /// \brief An enumerated integer property defining the start pixel for a camera delivering an unconverted bayer image.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBayerMosaicParity.
     */
    PropertyIBayerMosaicParity bayerParity;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getBitsPerPixel( void ) const
    {
        return bitsPerPixel;
    }
    PropertyICameraDataFormat getFormat( void ) const
    {
        return format;
    }
    PropertyIBayerMosaicParity getBayerParity( void ) const
    {
        return bayerParity;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class to describe a SDI(Serial Digital Interface) camera signal (\b Device specific interface layout only).
/**
 *  This class provides access to all settings specific for SDI video signals.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionSDI : public CameraDescriptionDigitalBase
//-----------------------------------------------------------------------------
{
    friend class CameraDescriptionManager;
    explicit CameraDescriptionSDI( HDRV hDrv, HLIST hList ) : CameraDescriptionDigitalBase( hDrv, hList ), videoStandard()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( videoStandard, "VideoStandard" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property for defining the video standard this camera is compliant with.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TVideoStandard.
     */
    PropertyIVideoStandard videoStandard;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIVideoStandard getVideoStandard( void ) const
    {
        return videoStandard;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A base class to describe an arbitrary digital camera signal(\b Device specific interface layout only).
/**
 *  This class provides access to all settings specific for digital video signals.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionDigitalBase2 : public CameraDescriptionDigitalBase
//-----------------------------------------------------------------------------
{
protected:
    explicit CameraDescriptionDigitalBase2( HDRV hDrv, HLIST hList ) : CameraDescriptionDigitalBase( hDrv, hList ),
        pixelsPerCycle(), scanMode()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( pixelsPerCycle, "PixelsPerCycle" );
        locator.bindComponent( scanMode, "ScanMode" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property defining the number of pixels per clock cycle transmitted by this camera.
    /**
     *  This corresponds the number of taps used by the camera in the described configuration.
     */
    PropertyI pixelsPerCycle;
    /// \brief An enumerated integer property providing information about the sensor of this camera.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraScanMode.
     */
    PropertyICameraScanMode scanMode;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getPixelsPerCycle( void ) const
    {
        return pixelsPerCycle;
    }
    PropertyICameraScanMode getScanMode( void ) const
    {
        return scanMode;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to describe a CameraLink&reg; compliant camera(\b Device specific interface layout only).
/**
 *  This class provides access to all settings specific for CameraLink&reg; compliant cameras.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionCameraLink : public CameraDescriptionDigitalBase2
//-----------------------------------------------------------------------------
{
    friend class CameraDescriptionManager;
    explicit CameraDescriptionCameraLink( HDRV hDrv, HLIST hList ) : CameraDescriptionDigitalBase2( hDrv, hList ), dataValid(),
        tapsXGeometry(), tapsYGeometry()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( dataValid, "DataValid" );
        locator.bindComponent( tapsXGeometry, "TapsXGeometry" );
        locator.bindComponent( tapsYGeometry, "TapsYGeometry" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the way this camera transmits the data valid (\b DVAL) signal.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraLinkDataValidMode.
     */
    PropertyICameraLinkDataValidMode dataValid;
    /// \brief An enumerated integer property defining the way this camera transmits the pixel data in X direction.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTapsXGeometry.
     */
    PropertyICameraTapsXGeometry tapsXGeometry;
    /// \brief An enumerated integer property defining the way this camera transmits the pixel data in Y direction.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraTapsYGeometry.
     */
    PropertyICameraTapsYGeometry tapsYGeometry;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyICameraLinkDataValidMode getDataValid( void ) const
    {
        return dataValid;
    }
    PropertyICameraTapsXGeometry getTapsXGeometry( void ) const
    {
        return tapsXGeometry;
    }
    PropertyICameraTapsYGeometry getTapsYGeometry( void ) const
    {
        return tapsYGeometry;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class to describe a non-standard digital video signal(\b Device specific interface layout only).
/**
 *  This class provides access to all settings specific for non-standard digital video signals.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionDigital : public CameraDescriptionDigitalBase2
//-----------------------------------------------------------------------------
{
    friend class CameraDescriptionManager;
    explicit CameraDescriptionDigital( HDRV hDrv, HLIST hList ) : CameraDescriptionDigitalBase2( hDrv, hList ),
        frameSync(), lineSync(), pixelClk()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( frameSync, "FrameSync" );
        locator.bindComponent( lineSync, "LineSync" );
        locator.bindComponent( pixelClk, "PixelClk" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the edge valid for the frame sync signal of this camera.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     *
     *  \note
     *  Not every value defined by this enumeration will be supported by this property. Check which values
     *  are available by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictValue
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyICameraExternalSyncEdge frameSync;
    /// \brief An enumerated integer property defining the edge valid for the line sync. of this camera.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     *
     *  \note
     *  Not every value defined by this enumeration will be supported by this property. Check which values
     *  are available by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictValue
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyICameraExternalSyncEdge lineSync;
    /// \brief An enumerated integer property defining the edge valid for the pixel clock of this camera.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     *
     *  \note
     *  Not every value defined by this enumeration will be supported by this property. Check which values
     *  are available by reading the properties translation dictionary with
     *  the functions \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictString and
     *  \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDictValue
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  C++ offers the more efficient function \b mvIMPACT::acquire::PropertyICameraExternalSyncEdge::getTranslationDict
     *  in addition to the functions mentioned above.
     *  \endif
     */
    PropertyICameraExternalSyncEdge pixelClk;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyICameraExternalSyncEdge getFrameSync( void ) const
    {
        return frameSync;
    }
    PropertyICameraExternalSyncEdge getLineSync( void ) const
    {
        return lineSync;
    }
    PropertyICameraExternalSyncEdge getPixelClk( void ) const
    {
        return pixelClk;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief An internal base class to describe standard analogue video cameras(\b Device specific interface layout only).
/**
 *  Instances of this class can't be constructed directly. Use one of the derived types.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionStandardBase : public CameraDescriptionBase
//-----------------------------------------------------------------------------
{
protected:
    explicit CameraDescriptionStandardBase( HDRV hDrv, HLIST hList ) : CameraDescriptionBase( hDrv, hList ), videoStandard(),
        scanStandard(), scanRate_kHz(), startField()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( videoStandard, "VideoStandard" );
        locator.bindComponent( scanStandard, "ScanStandard" );
        locator.bindComponent( scanRate_kHz, "ScanRate_kHz" );
        locator.bindComponent( startField, "StartField" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property for defining the video standard this camera is compliant with.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TVideoStandard.
     */
    PropertyIVideoStandard videoStandard;
    /// \brief An enumerated integer property defining the scan standard this camera is compliant with.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TScanStandard.
     */
    PropertyIScanStandard scanStandard;
    /// \brief An integer property containing the scan rate in kHz needed to driver the AD converter of the capture device with to convert the video signal properly.
    PropertyI scanRate_kHz;
    /// \brief An enumerated integer property containing information about the first field transmitted by a camera.
    /**
     *  A camera might transmit either the odd or the even field of an interlaced video signal first.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TAcquisitionField.
     */
    PropertyIAcquisitionField startField;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIVideoStandard getVideoStandard( void ) const
    {
        return videoStandard;
    }
    PropertyIScanStandard getScanStandard( void ) const
    {
        return scanStandard;
    }
    PropertyI getScanRate_kHz( void ) const
    {
        return scanRate_kHz;
    }
    PropertyIAcquisitionField getStartField( void ) const
    {
        return startField;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief A class describing analogue standard compliant video cameras(\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionStandard : public CameraDescriptionStandardBase
//-----------------------------------------------------------------------------
{
    friend class CameraDescriptionManager;
    explicit CameraDescriptionStandard( HDRV hDrv, HLIST hList ) : CameraDescriptionStandardBase( hDrv, hList ) {}
};

//-----------------------------------------------------------------------------
/// \brief A class describing non-standard video cameras(\b Device specific interface layout only).
/**
 *  This class can be used to describe a non-standard analogue video camera. A camera
 *  considered as non-standard when it is not compliant with one of the defined
 *  video standards like e.g. CCIR.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionNonStandard : public CameraDescriptionStandardBase
//-----------------------------------------------------------------------------
{
    friend class CameraDescriptionManager;
    explicit CameraDescriptionNonStandard( HDRV hDrv, HLIST hList ) : CameraDescriptionStandardBase( hDrv, hList ), lineFrequency_Hz(),
        VDOutput(), HDOutput(), syncOutput(), pixelClockOutput(), fieldGateMode(), fieldGateStart(), fieldGateWidth(),
        clampMode(), clampStart_us()
    {
        ComponentLocator locator( m_hRoot );
        locator.bindComponent( lineFrequency_Hz, "LineFrequency_Hz" );
        locator.bindComponent( VDOutput, "VDOutput" );
        locator.bindComponent( HDOutput, "HDOutput" );
        locator.bindComponent( syncOutput, "SyncOutput" );
        locator.bindComponent( pixelClockOutput, "PixelClockOutput" );
        locator.bindComponent( fieldGateMode, "FieldGateMode" );
        locator.bindComponent( fieldGateStart, "FieldGateStart" );
        locator.bindComponent( fieldGateWidth, "FieldGateWidth" );
        locator.bindComponent( clampMode, "ClampMode" );
        locator.bindComponent( clampStart_us, "ClampStart_us" );
    }
public:
    PYTHON_ONLY( %immutable; )
    /// \brief An integer property containing the line frequency of this camera in Hertz.
    PropertyI lineFrequency_Hz;
    /// \brief An enumerated integer property defining whether the vertical sync. information is part of the video signal or transmitted via a separate wire.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     */
    PropertyICameraExternalSyncEdge VDOutput;
    /// \brief An enumerated integer property defining whether the horizontal sync. information is part of the video signal or transmitted via a separate wire.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     */
    PropertyICameraExternalSyncEdge HDOutput;
    /// \brief An enumerated integer property for RGB cameras containing information where the sync. information is transmitted.
    /**
     *  This can either be within one of the three color components or on a separate wire.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncOutput.
     */
    PropertyICameraExternalSyncOutput syncOutput;
    /// \brief An enumerated integer property containing information whether the pixel clock is part of the video signal or not.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TCameraExternalSyncEdge.
     */
    PropertyICameraExternalSyncEdge pixelClockOutput;
    /// \brief An enumerated integer property defining the clamp mode for this camera description.
    /**
     *  This defines how the field detection(ODD/EVEN) is done.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TFieldGateMode.
     */
    PropertyIFieldGateMode fieldGateMode;
    /// \brief An integer property defining the start position (in pixel clocks) of the field gate detection area.
    /**
     *  This will only need modification in very rare cases.
     */
    PropertyI fieldGateStart;
    /// \brief An integer property defining the width (in pixel clocks) of the field gate detection area.
    /**
     *  This will only need modification in very rare cases.
     */
    PropertyI fieldGateWidth;
    /// \brief An enumerated integer property defining the clamp mode for this camera description.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TClampMode.
     */
    PropertyIClampMode clampMode;
    /// \brief The clamp start position defines where in each line of the video signal the frame grabber assumes the black level position.
    /**
     *  This will only need modification in very rare cases.
     */
    PropertyI clampStart_us;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI                           getLineFrequency_Hz( void ) const
    {
        return lineFrequency_Hz;
    }
    PropertyICameraExternalSyncEdge     getVDOutput( void ) const
    {
        return VDOutput;
    }
    PropertyICameraExternalSyncEdge     getHDOutput( void ) const
    {
        return HDOutput;
    }
    PropertyICameraExternalSyncOutput   getSyncOutput( void ) const
    {
        return syncOutput;
    }
    PropertyICameraExternalSyncEdge     getPixelClockOutput( void ) const
    {
        return pixelClockOutput;
    }
    PropertyIFieldGateMode              getFieldGateMode( void ) const
    {
        return fieldGateMode;
    }
    PropertyI                           getFieldGateStart( void ) const
    {
        return fieldGateStart;
    }
    PropertyI                           getFieldGateWidth( void ) const
    {
        return fieldGateWidth;
    }
    PropertyIClampMode                  getClampMode( void ) const
    {
        return clampMode;
    }
    PropertyI                           getClampStart_us( void ) const
    {
        return clampStart_us;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Grants access to camera description objects(\b Device specific interface layout only).
/**
 *  This class provides access to the various camera description objects. Each
 *  device recognizes a different set of camera descriptions. E.g. a digital
 *  frame grabber will not be able to work with analogue cameras. Each camera description
 *  class will be derived from \b mvIMPACT::acquire::CameraDescriptionBase.
 *
 *  \note
 *  Please note that instances of this class will \b ONLY list camera descriptions
 *  that have once been selected by the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type
 *  at runtime. This is to save memory. A complete list of camera descriptions
 *  available for the current device therefore can only be queried by reading the
 *  the translation dictionary of the property \b mvIMPACT::acquire::CameraSettingsFrameGrabber::type.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  #include <mvIMPACT_CPP/mvIMPACT_acquire.h>
 *  #include <algorithm>
 *  #include <iostream>
 *
 *  using namespace std;
 *  using namespace mvIMPACT::acquire;
 *
 *  //-----------------------------------------------------------------------------
 *  template<class T>
 *  class DisplayDictEntry : public unary_function<pair<string, T>, void>
 *  //-----------------------------------------------------------------------------
 *  {
 *  public:
 *    void operator()( const pair<string, T>& data ) const
 *    {
 *      cout << "  [" << data.second << "]: " << data.first << endl;
 *    }
 *  };
 *
 *  //-----------------------------------------------------------------------------
 *  // lists all available camera descriptions for a certain device
 *  int main( int argc, char* argv[] )
 *  //-----------------------------------------------------------------------------
 *  {
 *    DeviceManager devMgr;
 *    Device* pDev = devMgr[0];
 *    if( !pDev )
 *    {
 *      cout << "No device found" << endl;
 *      return 0;
 *    }
 *
 *    // display the name of every camera descripiton available for this device.
 *    // this might be less then the number of camera descriptions available on the system as e.g.
 *    // an analog frame grabber can't use descriptions for digital cameras
 *    CameraSettingsFrameGrabber(pDev);
 *    IntDict vAvailableDescriptions;
 *    cs.type.getTranslationDict( vAvailableDescriptions );
 *    cout << endl << "Available camera descriptions: " << vAvailableDescriptions.size() << endl
 *         << "----------------------------------" << endl;
 *    for_each( vAvailableDescriptions.begin(), vAvailableDescriptions.end(), DisplayDictEntry<int>() )
 *
 *    // list all descriptions that already have been selected at least once
 *    CameraDescriptionManager camMgr(pDev);
 *    unsigned int camCnt = camMgr.getCLCameraDescriptionCount();
 *    cout << "Available CL descriptions: " << camCnt << endl;
 *    for( unsigned int a=0; a<camCnt; a++ )
 *    {
 *      cout << "  " << camMgr.cameraDescriptionCameraLink(a)->name.read() << endl;
 *    }
 *    camCnt = camMgr.getStandardCameraDescriptionCount();
 *    cout << "Available Std descriptions: " << camCnt << endl;
 *    for( unsigned int b=0; b<camCnt; b++ )
 *    {
 *      cout << "  " << camMgr.cameraDescriptionStandard(b)->name.read() << endl;
 *    }
 *    camCnt = camMgr.getNonStandardCameraDescriptionCount();
 *    cout << "Available NonStd descriptions: " << camCnt << endl;
 *    for( unsigned int c=0; c<camCnt; c++ )
 *    {
 *      cout << "  " << camMgr.cameraDescriptionNonStandard(c)->name.read() << endl;
 *    }
 *    return 0;
 *  }
 * \endcode
 *  \endif
 *
 *  \note
 *  This class will only be available for frame grabber devices. For other devices
 *  the class constructor will raise an exception.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class CameraDescriptionManager
//-----------------------------------------------------------------------------
{
    typedef std::map<std::string, unsigned int> StringUIntMap;
#           ifndef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    struct ReferenceCountedData
            //-----------------------------------------------------------------------------
    {
        HDRV                                               m_hDrv;
        HLIST                                              m_hListCameraDescriptions;
        mutable unsigned int                               m_lastListSize;
        mutable std::vector<CameraDescriptionStandard*>    m_vStdDescriptions;
        mutable std::vector<CameraDescriptionNonStandard*> m_vNonStdDescriptions;
        mutable std::vector<CameraDescriptionSDI*>         m_vSDIDescriptions;
        mutable std::vector<CameraDescriptionCameraLink*>  m_vCLDescriptions;
        mutable std::vector<CameraDescriptionDigital*>     m_vDigitalDescriptions;
        mutable StringUIntMap                              m_mListNameToDescription;
        mutable StringUIntMap                              m_mStdNameToDescription;
        mutable StringUIntMap                              m_mNonStdNameToDescription;
        mutable StringUIntMap                              m_mSDINameToDescription;
        mutable StringUIntMap                              m_mCLNameToDescription;
        mutable StringUIntMap                              m_mDigitalNameToDescription;
        unsigned int                                       m_refCnt;
        ReferenceCountedData( HDRV hDrv, HLIST hList ) : m_hDrv( hDrv ), m_hListCameraDescriptions( hList ),
            m_lastListSize( 0 ), m_vStdDescriptions(), m_vNonStdDescriptions(), m_vSDIDescriptions(), m_vCLDescriptions(),
            m_vDigitalDescriptions(), m_mListNameToDescription(), m_mStdNameToDescription(),
            m_mNonStdNameToDescription(), m_mCLNameToDescription(), m_mDigitalNameToDescription(),
            m_refCnt( 1 ) {}
        ~ReferenceCountedData()
        {
            std::vector<CameraDescriptionStandard*>::size_type vStdSize = m_vStdDescriptions.size();
            for( std::vector<CameraDescriptionStandard*>::size_type a = 0; a < vStdSize; a++ )
            {
                delete m_vStdDescriptions[a];
            }
            std::vector<CameraDescriptionStandard*>::size_type vNonStdSize = m_vNonStdDescriptions.size();
            for( std::vector<CameraDescriptionStandard*>::size_type b = 0; b < vNonStdSize; b++ )
            {
                delete m_vNonStdDescriptions[b];
            }
            std::vector<CameraDescriptionSDI*>::size_type vSDISize = m_vSDIDescriptions.size();
            for( std::vector<CameraDescriptionSDI*>::size_type c = 0; c < vSDISize; c++ )
            {
                delete m_vSDIDescriptions[c];
            }
            std::vector<CameraDescriptionStandard*>::size_type vCLSize = m_vCLDescriptions.size();
            for( std::vector<CameraDescriptionStandard*>::size_type d = 0; d < vCLSize; d++ )
            {
                delete m_vCLDescriptions[d];
            }
            std::vector<CameraDescriptionStandard*>::size_type vDigSize = m_vDigitalDescriptions.size();
            for( std::vector<CameraDescriptionStandard*>::size_type e = 0; e < vDigSize; e++ )
            {
                delete m_vDigitalDescriptions[e];
            }
        }
    }* m_pRefData;
#           endif // #ifdef DOXYGEN_SHOULD_SKIP_THIS
    //-----------------------------------------------------------------------------
    void dealloc( void )
    //-----------------------------------------------------------------------------
    {
        --( m_pRefData->m_refCnt );
        if( m_pRefData->m_refCnt == 0 )
        {
            delete m_pRefData;
        }
    }
    //-----------------------------------------------------------------------------
    bool locateDescription( const StringUIntMap& m, const std::string& name, unsigned int& index ) const
    //-----------------------------------------------------------------------------
    {
        update();
        StringUIntMap::const_iterator it = m_pRefData->m_mListNameToDescription.find( name );
        if( it != m_pRefData->m_mListNameToDescription.end() )
        {
            index = it->second;
            return true;
        }
        else
        {
            it = m.find( name );
            if( it != m.end() )
            {
                index = it->second;
                return true;
            }
        }
        return false;
    }
    //-----------------------------------------------------------------------------
    void update( void ) const
    //-----------------------------------------------------------------------------
    {
        ComponentList camDescList( m_pRefData->m_hListCameraDescriptions );
        unsigned int curSize = camDescList.size();
        if( m_pRefData->m_lastListSize != curSize )
        {
            // new cameras at runtime can only appear at the end of the list
            ComponentIterator it( m_pRefData->m_hListCameraDescriptions );
            it = it.firstChild();
            // move to the old end
            for( unsigned int i = 0; ( i < m_pRefData->m_lastListSize ) && it.isValid(); i++ )
            {
                ++it;
            }
            // start to add new descriptions
            while( it.isValid() )
            {
                ComponentList list( it );
                const std::string contentDescriptor( list.contentDescriptor() );
                if( contentDescriptor == "CameraLink" )
                {
                    CameraDescriptionCameraLink* p = new CameraDescriptionCameraLink( m_pRefData->m_hDrv, list );
                    unsigned int index = static_cast<unsigned int>( m_pRefData->m_vCLDescriptions.size() ); // do not change order of this and the next line!
                    m_pRefData->m_vCLDescriptions.push_back( p );
                    m_pRefData->m_mListNameToDescription.insert( std::make_pair( list.name(), index ) );
                    m_pRefData->m_mCLNameToDescription.insert( std::make_pair( p->name.read(), index ) );
                }
                else if( contentDescriptor == "SDI" )
                {
                    CameraDescriptionSDI* p = new CameraDescriptionSDI( m_pRefData->m_hDrv, list );
                    unsigned int index = static_cast<unsigned int>( m_pRefData->m_vCLDescriptions.size() ); // do not change order of this and the next line!
                    m_pRefData->m_vSDIDescriptions.push_back( p );
                    m_pRefData->m_mListNameToDescription.insert( std::make_pair( list.name(), index ) );
                    m_pRefData->m_mSDINameToDescription.insert( std::make_pair( p->name.read(), index ) );
                }
                else if( contentDescriptor == "Digital" )
                {
                    CameraDescriptionDigital* p = new CameraDescriptionDigital( m_pRefData->m_hDrv, list );
                    unsigned int index = static_cast<unsigned int>( m_pRefData->m_vDigitalDescriptions.size() ); // do not change order of this and the next line!
                    m_pRefData->m_vDigitalDescriptions.push_back( p );
                    m_pRefData->m_mListNameToDescription.insert( std::make_pair( list.name(), index ) );
                    m_pRefData->m_mDigitalNameToDescription.insert( std::make_pair( p->name.read(), index ) );
                }
                else if( contentDescriptor == "Standard" )
                {
                    CameraDescriptionStandard* p = new CameraDescriptionStandard( m_pRefData->m_hDrv, list );
                    unsigned int index = static_cast<unsigned int>( m_pRefData->m_vStdDescriptions.size() ); // do not change order of this and the next line!
                    m_pRefData->m_vStdDescriptions.push_back( p );
                    m_pRefData->m_mListNameToDescription.insert( std::make_pair( list.name(), index ) );
                    m_pRefData->m_mStdNameToDescription.insert( std::make_pair( p->name.read(), index ) );
                }
                else if( contentDescriptor == "NonStandard" )
                {
                    CameraDescriptionNonStandard* p = new CameraDescriptionNonStandard( m_pRefData->m_hDrv, list );
                    unsigned int index = static_cast<unsigned int>( m_pRefData->m_vNonStdDescriptions.size() ); // do not change order of this and the next line!
                    m_pRefData->m_vNonStdDescriptions.push_back( p );
                    m_pRefData->m_mListNameToDescription.insert( std::make_pair( list.name(), index ) );
                    m_pRefData->m_mNonStdNameToDescription.insert( std::make_pair( p->name.read(), index ) );
                }
                else
                {
                    ; // NOT recognized....
                }
                ++it;
            }
            m_pRefData->m_lastListSize = curSize;
        }
    }
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraDescriptionManager object.
    explicit CameraDescriptionManager(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::>DeviceManager object.
        Device* pDev ) : m_pRefData( 0 )
    {
        if( !pDev->isOpen() )
        {
            pDev->open();
        }

        TDMR_ERROR result;
        HLIST hList;
        if( ( result = DMR_FindList( pDev->hDrv(), 0, dmltCameraDescriptions, 0, &hList ) ) != DMR_NO_ERROR )
        {
            ExceptionFactory::raiseException( MVIA_FUNCTION, __LINE__, result, INVALID_ID, "Couldn't find camera description list (is this a frame grabber?)" );
        }
        m_pRefData = new ReferenceCountedData( pDev->hDrv(), hList );
    }
    /// \brief Constructs a new \b mvIMPACT::acquire::CameraDescriptionManager from and exisiting one.
    explicit CameraDescriptionManager(  /// A constant reference to the \b mvIMPACT::acquire::CameraDescriptionManager object, this object shall be created from
        const CameraDescriptionManager& src ) : m_pRefData( src.m_pRefData )
    {
        ++( m_pRefData->m_refCnt );
    }
    /// \brief Class destructor
    ~CameraDescriptionManager()
    {
        dealloc();
    }
#           ifndef WRAP_PYTHON
    /// \brief Allows assignments of \b mvIMPACT::acquire::CameraDescriptionManager objects
    CameraDescriptionManager& operator=( const CameraDescriptionManager& rhs )
    {
        if( this != &rhs )
        {
            dealloc();
            m_pRefData = rhs.m_pRefData;
            // inc. the NEW reference count
            ++m_pRefData->m_refCnt;
        }
        return *this;
    }
#           endif // #ifndef WRAP_PYTHON (In Python, object assignment amounts to just a reference count increment anyhow; you need to call the constructor or possibly some slice operation to make a true copy)
    /// \brief Returns the number of CameraLink&reg; camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getCLCameraDescriptionCount( void ) const
    {
        update();
        return static_cast<unsigned int>( m_pRefData->m_vCLDescriptions.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionCameraLink object specifying the camera description found at the given index in the camera description managers internal list.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of CameraLink&reg; camera descriptions \n
     *  detected for this device a STL out_of_range exception will be thrown.
     *  \endif
     */
    CameraDescriptionCameraLink* cameraDescriptionCameraLink(
        /// [in] The index of the camera description to return
        unsigned int index ) const
    {
        update();
        return m_pRefData->m_vCLDescriptions.at( index );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionCameraLink object specifying the camera description with the given name in the camera description managers internal list.
    /**
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::CameraDescriptionCameraLink object specifying the camera
     *  description with the given name in the camera description managers internal list if a camera
     *  description with \a name is available.
     *  - an invalid pointer or reference otherwise.
     */
    CameraDescriptionCameraLink* cameraDescriptionCameraLink(
        /// [in] The name of the camera description. This can either be
        /// the name of the list like e.g. 'CameraLink_Generic' or the
        /// value of the property 'name' of this description like
        /// e.g. 'Generic'.
        const std::string& name ) const
    {
        unsigned int index = 0;
        return ( ( locateDescription( m_pRefData->m_mCLNameToDescription, name, index ) == true ) ? m_pRefData->m_vCLDescriptions.at( index ) : 0 );
    }
    /// \brief Returns the number of SDI camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getSDICameraDescriptionCount( void ) const
    {
        update();
        return static_cast<unsigned int>( m_pRefData->m_vSDIDescriptions.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionSDI object specifying the camera description found at the given index in the camera description managers internal list.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of SDI camera descriptions \n
     *  detected for this device a STL out_of_range exception will be thrown.
     *  \endif
     */
    CameraDescriptionSDI* cameraDescriptionSDI(
        /// [in] The index of the camera description to return
        unsigned int index ) const
    {
        update();
        return m_pRefData->m_vSDIDescriptions.at( index );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionSDI object specifying the camera description with the given name in the camera description managers internal list.
    /**
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::CameraDescriptionSDI object specifying the camera
     *  description with the given name in the camera description managers internal list if a camera
     *  description with \a name is available.
     *  - an invalid pointer or reference otherwise.
     */
    CameraDescriptionSDI* cameraDescriptionSDI(
        /// [in] The name of the camera description. This can either be
        /// the name of the list like e.g. 'CameraLink_Generic' or the
        /// value of the property 'name' of this description like
        /// e.g. 'Generic'.
        const std::string& name ) const
    {
        unsigned int index = 0;
        return ( ( locateDescription( m_pRefData->m_mSDINameToDescription, name, index ) == true ) ? m_pRefData->m_vSDIDescriptions.at( index ) : 0 );
    }
    /// \brief Returns the number of non-standard digital camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getDigitalCameraDescriptionCount( void ) const
    {
        update();
        return static_cast<unsigned int>( m_pRefData->m_vDigitalDescriptions.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionDigital object specifying the camera description found at the given index in the camera description managers internal list.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of non-standard digital camera descriptions \n
     *  detected for this device a STL out_of_range exception will be thrown.
     *  \endif
     */
    CameraDescriptionDigital* cameraDescriptionDigital(
        /// [in] The index of the camera description to return
        unsigned int index ) const
    {
        update();
        return m_pRefData->m_vDigitalDescriptions.at( index );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionDigital object specifying the camera description with the given name in the camera description managers internal list.
    /**
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::CameraDescriptionDigital object specifying the camera
     *  description with the given name in the camera description managers internal list if a camera
     *  description with \a name is available.
     *  - an invalid pointer or reference otherwise.
     */
    CameraDescriptionDigital* cameraDescriptionDigital(
        /// [in] The name of the camera description. This can either be
        /// the name of the list like e.g. 'Digital_Generic' or the
        /// value of the property 'name' of this description like
        /// e.g. 'Generic'.
        const std::string& name ) const
    {
        unsigned int index = 0;
        return ( ( locateDescription( m_pRefData->m_mDigitalNameToDescription, name, index ) == true ) ? m_pRefData->m_vDigitalDescriptions.at( index ) : 0 );
    }
    /// \brief Returns the number of standard camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getStandardCameraDescriptionCount( void ) const
    {
        update();
        return static_cast<unsigned int>( m_pRefData->m_vStdDescriptions.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionStandard object specifying the camera description found at the given index in the camera description managers internal list.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of standard camera descriptions \n
     *  detected for this device a STL out_of_range exception will be thrown.
     *  \endif
     */
    CameraDescriptionStandard* cameraDescriptionStandard(
        /// [in] The index of the camera description to return
        unsigned int index ) const
    {
        update();
        return m_pRefData->m_vStdDescriptions.at( index );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionStandard object specifying the camera description with the given name in the camera description managers internal list.
    /**
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::CameraDescriptionStandard object specifying the camera
     *  description with the given name in the camera description managers internal list if a camera
     *  description with \a name is available.
     *  - an invalid pointer or reference otherwise.
     */
    CameraDescriptionStandard* cameraDescriptionStandard(
        /// [in] The name of the camera description. This can either be
        /// the name of the list like e.g. 'Standard_Generic' or the
        /// value of the property 'name' of this description like
        /// e.g. 'Generic'.
        const std::string& name ) const
    {
        unsigned int index = 0;
        return ( ( locateDescription( m_pRefData->m_mStdNameToDescription, name, index ) == true ) ? m_pRefData->m_vStdDescriptions.at( index ) : 0 );
    }
    /// \brief Returns the number of non-standard camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getNonStandardCameraDescriptionCount( void ) const
    {
        update();
        return static_cast<unsigned int>( m_pRefData->m_vNonStdDescriptions.size() );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionNonStandard object specifying the camera description found at the given index in the camera description managers internal list.
    /**
     *  \if DOXYGEN_CPP_DOCUMENTATION
     *  If the index is either equal or higher than the number of non-standard camera descriptions \n
     *  detected for this device a STL out_of_range exception will be thrown.
     *  \endif
     */
    CameraDescriptionNonStandard* cameraDescriptionNonStandard(
        /// [in] The index of the camera description to return
        unsigned int index ) const
    {
        update();
        return m_pRefData->m_vNonStdDescriptions.at( index );
    }
    /// \brief Returns a pointer to a \b mvIMPACT::acquire::CameraDescriptionNonStandard object specifying the camera description with the given name in the camera description managers internal list.
    /**
     *  \return
     *  - a pointer to a \b mvIMPACT::acquire::CameraDescriptionNonStandard object specifying the camera
     *  description with the given name in the camera description managers internal list if a camera
     *  description with \a name is available.
     *  - an invalid pointer or reference otherwise.
     */
    CameraDescriptionNonStandard* cameraDescriptionNonStandard(
        /// [in] The name of the camera description. This can either be
        /// the name of the list like e.g. 'NonStandard_Generic' or the
        /// value of the property 'name' of this description like
        /// e.g. 'Generic'.
        const std::string& name ) const
    {
        unsigned int index = 0;
        return ( ( locateDescription( m_pRefData->m_mNonStdNameToDescription, name, index ) == true ) ? m_pRefData->m_vNonStdDescriptions.at( index ) : 0 );
    }
    /// \brief Returns the total number camera descriptions currently available for the device that constructed this instance of the class.
    unsigned int getTotalCameraDescriptionCount( void ) const
    {
        update();
        return m_pRefData->m_lastListSize;
    }
};
#   endif // #ifndef #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief Properties for accessing features belonging to the I2C control(\b Device specific interface layout only).
/**
 *  Properties in this class will only be available if a device has a local I2C
 *  bus and this is configured for access from an application. Right now this is only
 *  the case for mvBlueFOX-MLC devices.
 *
 *  \b mvBlueFOX \b specific:
 *
 *  For mvBlueFOX devices
 *  - clock stretching is supported
 *  - bus master is supported
 *
 *  The following I2C addresses will be blocked for access from an application:
 *  <table>
 *  <tr><td class="header">i2c address</td><td class="header">remark</td></tr>
 *  <tr><td class="indexvalue">0x20-0x3F</td><td class="indexvalue">-</td></tr>
 *  <tr><td class="indexvalue">0x66-0x67</td><td class="indexvalue">-</td></tr>
 *  <tr><td class="indexvalue">0x90-0x91</td><td class="indexvalue">mvBlueFOX-200w only</td></tr>
 *  <tr><td class="indexvalue">0xA0-0xA3</td><td class="indexvalue">-</td></tr>
 *  <tr><td class="indexvalue">0xA6-0xA7</td><td class="indexvalue">-</td></tr>
 *  <tr><td class="indexvalue">0xBA-0xBB</td><td class="indexvalue">mvBlueFOX-202a and mvBlueFOX-205 only</td></tr>
 *  </table>
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  I2CControl i2cc(pBF);
 *  if( i2cc.I2COperationMode.isValid() )
 *  {
 *    // direct property access
 *    i2cc.I2CBufferLength.write( 0 );
 *    i2cc.I2COperationMode.write( I2ComRead );
 *    assert( ( i2cc.I2COperationExecute.call() == DMR_INVALID_PARAMETER ) && "Unexpected driver behaviour" );
 *    assert( ( i2cc.I2COperationStatus.read() == I2CosNotEnoughData ) && "Unexpected driver behaviour" );
 *    i2cc.I2CBufferLength.write( 1 );
 *    // assuming we write to an invalid address
 *    assert( ( i2cc.I2COperationExecute.call() == DMR_EXECUTION_FAILED ) && "Unexpected driver behaviour" );
 *    assert( ( i2cc.I2COperationStatus.read() == I2CosFailure ) && "Unexpected driver behaviour" );
 *    i2cc.I2COperationMode.write( I2ComWrite );
 *    i2cc.I2CBuffer.writeBinary( string() );
 *    assert( ( i2cc.I2COperationExecute.call() == DMR_INVALID_PARAMETER ) && "Unexpected driver behaviour" );
 *    assert( ( i2cc.I2COperationStatus.read() == I2CosNotEnoughData ) && "Unexpected driver behaviour" );
 *    char binData[2] = { 'A', 'B' };
 *    i2cc.I2CBuffer.writeBinary( string(binData, sizeof(binData)) );
 *    // assuming we write to an invalid address
 *    assert( ( i2cc.I2COperationExecute.call() == DMR_EXECUTION_FAILED ) && "Unexpected driver behaviour" );
 *    assert( ( i2cc.I2COperationStatus.read() == I2CosFailure ) && "Unexpected driver behaviour" );
 *    // Write some data. This will only work if serveral conditions are met:
 *    // - there is a device that can be written to at address 0xA6
 *    // - the sub-address 0x04 is valid
 *    // - the device is designed to work with 8 bit sub-addresses
 *    // - the device can deal with 9 bytes in a single command
 *    i2cc.I2CDeviceAddress.write( 0xA6 );
 *    i2cc.I2CDeviceSubAddress.write( 0x04 );
 *    i2cc.I2CDeviceSubAddressWidth.write( 8 );
 *    char binData[9] = { 'D', 'E', 'A', 'D', ' ', 'B', 'E', 'E', 'F' };
 *    i2cc.I2CBuffer.writeBinary( string(binData, sizeof(binData)) );
 *    i2cc.I2COperationMode.write( I2ComWrite );
 *    int I2COperationExecuteResult = i2cc.I2COperationExecute.call();
 *    if( I2COperationExecuteResult != DMR_NO_ERROR )
 *    {
 *      printf( "'I2COperationExecute' write failed. Return value: %s(%d).\n", ImpactAcquireException::getErrorCodeAsString( I2COperationExecuteResult ).c_str(), I2COperationExecuteResult );
 *    }
 *    printf( "'I2COperationStatus' after write: %s.\n", i2cc.I2COperationStatus.readS().c_str() );
 *    // Read some data. Similar condition as for write apply
 *    const int bytesToRead = 4;
 *    i2cc.I2CDeviceAddress.write( 0xA8 );
 *    i2cc.I2CDeviceSubAddress.write( 0x00 );
 *    i2cc.I2CDeviceSubAddressWidth.write( 8 );
 *    i2cc.I2CBufferLength.write( bytesToRead ); // read 'bytesToRead' bytes
 *    i2cc.I2COperationMode.write( I2ComRead );
 *    i2cc.I2COperationExecute.call();
 *    I2COperationExecuteResult = i2cc.I2COperationExecute.call();
 *    if( I2COperationExecuteResult != DMR_NO_ERROR )
 *    {
 *      printf( "'I2COperationExecute' read failed. Return value: %s(%d).\n", ImpactAcquireException::getErrorCodeAsString( I2COperationExecuteResult ).c_str(), I2COperationExecuteResult );
 *    }
 *    printf( "'I2COperationStatus' after read: %s.\n", i2cc.I2COperationStatus.readS().c_str() );
 *    if( i2cc.I2CBuffer.binaryDataBufferSize() != bytesToRead )
 *    {
 *      printf( "'I2CBuffer' reports %d bytes of data while %d bytes where expected.\n", i2cc.I2CBuffer.binaryDataBufferSize(), bytesToRead );
 *    }
 *    // usage of the convenience functions
 *    i2cc.I2CWrite( 0xA4, 0x00, 8, string("TEST") );
 *    const string i2cReadBuffer = i2cc.I2CRead( 0xA4, 0x00, 8, 4 );
 *  }
 *  else
 *  {
 *    printf( "I2CControl not available.\n" );
 *  }
 * \endcode
 *  \endif
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class I2CControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
private:
    void prepareI2CAccess( int deviceAddress, int deviceSubAddress, int deviceSubAddressWidth, TI2COperationMode mode )
    {
        I2CDeviceAddress.write( deviceAddress );
        I2CDeviceSubAddress.write( deviceSubAddress );
        I2CDeviceSubAddressWidth.write( deviceSubAddressWidth );
        I2COperationMode.write( mode );
    }
public:
    /// brief Constructs a new \b mvIMPACT::acquire::I2CControl object.
    explicit I2CControl(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ): ComponentCollection( pDev ), I2COperationMode(), I2COperationExecute(), I2COperationStatus(),
        I2CDeviceAddress(), I2CDeviceSubAddressWidth(), I2CDeviceSubAddress(), I2CBuffer(), I2CBufferLength()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        HLIST hList = locator.findComponent( "I2CControl" );
        if( hList != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "I2CControl" );
            m_hRoot = locator.searchbase_id();
            locator.bindComponent( I2COperationMode, "I2COperationMode" );
            locator.bindComponent( I2COperationExecute, "I2COperationExecute@i" );
            locator.bindComponent( I2COperationStatus, "I2COperationStatus" );
            locator.bindComponent( I2CDeviceAddress, "I2CDeviceAddress" );
            locator.bindComponent( I2CDeviceSubAddressWidth, "I2CDeviceSubAddressWidth" );
            locator.bindComponent( I2CDeviceSubAddress, "I2CDeviceSubAddress" );
            locator.bindComponent( I2CBuffer, "I2CBuffer" );
            locator.bindComponent( I2CBufferLength, "I2CBufferLength" );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property to select the I2C operation.
    /**
     *  The selected operation is executed when \b mvIMPACT::acquire::I2CControl::I2COperationExecute is called.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TI2COperationMode.
     */
    PropertyII2COperationMode I2COperationMode;
    /// \brief Calling this function will execute the operation selected by \b mvIMPACT::acquire::I2CControl::I2COperationMode.
    Method I2COperationExecute;
    /// \brief Represents the I2C operation execution status.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TI2COperationStatus.
     */
    PropertyII2COperationStatus I2COperationStatus;
    /// \brief An integer property storing the address of the I2C device to communicate with.
    PropertyI I2CDeviceAddress;
    /// \brief An enumerated integer property storing the sub-address width(in bits) of the I2C device to communicate with.
    /**
     *  Valid values for this property are:
     *
     *  - 0
     *  - 8
     *  - 16
     *
     *  \note
     *  This property must be set to 0 for devices not supporting a sub-address.
     */
    PropertyI I2CDeviceSubAddressWidth;
    /// \brief An integer property storing the sub-address of the I2C device to communicate with.
    /**
     *  When \b mvIMPACT::acquire::I2CControl::I2CDeviceSubAddressWidth is set to 0, this property will be ignored.
     */
    PropertyI I2CDeviceSubAddress;
    /// \brief Defines the intermediate access buffer that allows the exchange of data between the I2C device and the application.
    /**
     *  This property can store binary data.
     */
    PropertyS I2CBuffer;
    /// \brief An integer property controlling the length of the mapping between the I2C device and the \b mvIMPACT::acquire::I2CControl::I2CBuffer property.
    PropertyI I2CBufferLength;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyII2COperationMode getI2COperationMode( void ) const
    {
        return I2COperationMode;
    }
    Method getI2COperationExecute( void ) const
    {
        return I2COperationExecute;
    }
    PropertyII2COperationStatus getI2COperationStatus( void ) const
    {
        return I2COperationStatus;
    }
    PropertyI getI2CDeviceAddress( void ) const
    {
        return I2CDeviceAddress;
    }
    PropertyI getI2CDeviceSubAddressWidth( void ) const
    {
        return I2CDeviceSubAddressWidth;
    }
    PropertyI getI2CDeviceSubAddress( void ) const
    {
        return I2CDeviceSubAddress;
    }
    PropertyS getI2CBuffer( void ) const
    {
        return I2CBuffer;
    }
    PropertyI getI2CBufferLength( void ) const
    {
        return I2CBufferLength;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
    /// \brief Read data from an I2C device.
    /**
     *  This is a convenience function that wraps the property access a little.
     *  In order to find out if the command has been executed successfully
     *  \b mvIMPACT::acquire::I2CControl::I2COperationStatus should be checked afterwards.
     */
    std::string I2CRead(
        /// [in] The address of the I2C device to communicate with.
        int deviceAddress,
        /// [in] The sub-address of the I2C device to communicate with.
        int deviceSubAddress,
        /// [in] The sub-address width(in bits) of the I2C device to communicate with.
        int deviceSubAddressWidth,
        /// [in] The amount of bytes to read.
        int byteCnt )
    {
        prepareI2CAccess( deviceAddress, deviceSubAddress, deviceSubAddressWidth, I2ComRead );
        I2CBufferLength.write( byteCnt );
        I2COperationExecute.call();
        return I2CBuffer.readBinary();
    }
    /// \brief Write data to a I2C device.
    /**
     *  This is a convenience function that wraps the property access a little.
     *  In order to find out if the command has been executed successfully
     *  \b mvIMPACT::acquire::I2CControl::I2COperationStatus should be checked afterwards.
     */
    void I2CWrite(
        /// [in] The address of the I2C device to communicate with.
        int deviceAddress,
        /// [in] The sub-address of the I2C device to communicate with.
        int deviceSubAddress,
        /// [in] The sub-address width(in bits) of the I2C device to communicate with.
        int deviceSubAddressWidth,
        /// [in] The data to write to the I2C device.
        const std::string& data )
    {
        prepareI2CAccess( deviceAddress, deviceSubAddress, deviceSubAddressWidth, I2ComWrite );
        I2CBuffer.writeBinary( data );
        I2COperationExecute.call();
    }
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

//-----------------------------------------------------------------------------
/// \brief Properties for configuring settings belonging to the motor focus control (\b Device specific interface layout only).
/**
 *  Properties in this class will only be available if a device is fitted with
 *  a focus motor.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class MotorFocusControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::MotorFocusControl object.
    explicit MotorFocusControl(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ): ComponentCollection( pDev ), motorFocusSendBuffer(), motorFocusReceiveBuffer(), motorFocusSend(),
        motorFocusIncrement(), motorFocusNear(), motorFocusFar(), motorFocusAbsolutePositionCurrent(),
        motorFocusAbsolutePositionDesired(), motorFocusMoveToAbsolutePositionDesired()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        HLIST hList = locator.findComponent( "MotorFocusControl" );
        if( hList != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "MotorFocusControl" );
            m_hRoot = locator.searchbase_id();
            locator.bindComponent( motorFocusSendBuffer, "MotorFocusSendBuffer" );
            locator.bindComponent( motorFocusReceiveBuffer, "MotorFocusReceiveBuffer" );
            locator.bindComponent( motorFocusSend, "MotorFocusSend@i" );
            locator.bindComponent( motorFocusIncrement, "MotorFocusIncrement" );
            locator.bindComponent( motorFocusNear, "MotorFocusNear@i" );
            locator.bindComponent( motorFocusFar, "MotorFocusFar@i" );
            locator.bindComponent( motorFocusAbsolutePositionCurrent, "MotorFocusAbsolutePositionCurrent" );
            locator.bindComponent( motorFocusAbsolutePositionDesired, "MotorFocusAbsolutePositionDesired" );
            locator.bindComponent( motorFocusMoveToAbsolutePositionDesired, "MotorFocusMoveToAbsolutePositionDesired@i" );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief A string property storing a command to be sent to the motor focus.
    /**
     *  To actually send the command, the function \b mvIMPACT::acquire::MotorFocusControl::motorFocusSend must be executed.
     */
    PropertyS motorFocusSendBuffer;
    /// \brief A string property \b (read-only) that will contain answers sent by the motor focus controller.
    PropertyS motorFocusReceiveBuffer;
    /// \brief Calling this function will send the value of \b mvIMPACT::acquire::MotorFocusControl::motorFocusSendBuffer to the hardware.
    /**
     *  Afterwards \b mvIMPACT::acquire::MotorFocusControl::motorFocusReceiveBuffer will contain the hardwares answer.
     */
    Method motorFocusSend;
    /// \brief An integer property storing an increment(in encoder counts) that will be used by subsequent calls to \b mvIMPACT::acquire::MotorFocusControl::motorFocusNear and \b mvIMPACT::acquire::MotorFocusControl::motorFocusFar commands.
    PropertyI motorFocusIncrement;
    /// \brief Calling this function will cause the motor focus to move forward by \b mvIMPACT::acquire::MotorFocusControl::motorFocusIncrement encoder units.
    Method motorFocusNear;
    /// \brief Calling this function will cause the motor focus to move backward by \b mvIMPACT::acquire::MotorFocusControl::motorFocusIncrement encoder units.
    Method motorFocusFar;
    /// \brief An integer property \b (read-only) storing the current absolute position(in encoder counts).
    PropertyI motorFocusAbsolutePositionCurrent;
    /// \brief An integer property storing an absolute position(in encoder counts) that will be used by subsequent calls to the \b mvIMPACT::acquire::MotorFocusControl::motorFocusMoveToAbsolutePositionDesired command.
    PropertyI motorFocusAbsolutePositionDesired;
    /// \brief Calling this function will cause the motor focus to move to the position defined by the value of \b mvIMPACT::acquire::MotorFocusControl::motorFocusAbsolutePositionDesired.
    Method motorFocusMoveToAbsolutePositionDesired;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    PropertyS getMotorFocusSendBuffer( void ) const
    {
        return motorFocusSendBuffer;
    }
    PropertyS getMotorFocusReceiveBuffer( void ) const
    {
        return motorFocusReceiveBuffer;
    }
    Method getMotorFocusSend( void ) const
    {
        return motorFocusSend;
    }
    PropertyI getMotorFocusIncrement( void ) const
    {
        return motorFocusIncrement;
    }
    Method getMotorFocusNear( void ) const
    {
        return motorFocusNear;
    }
    Method getMotorFocusFar( void ) const
    {
        return motorFocusFar;
    }
    PropertyI getMotorFocusAbsolutePositionCurrent( void ) const
    {
        return motorFocusAbsolutePositionCurrent;
    }
    PropertyI getMotorFocusAbsolutePositionDesired( void ) const
    {
        return motorFocusAbsolutePositionDesired;
    }
    Method getMotorFocusMoveToAbsolutePositionDesired( void ) const
    {
        return motorFocusMoveToAbsolutePositionDesired;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief Properties for configuring settings belonging to the digital I/O measurement(\b Device specific interface layout only).
/**
 *  Properties in this class will only be available if a device offers digital I/O measurement features.
 *
 *  \if DOXYGEN_CPP_DOCUMENTATION
 * \code
 *  // This code fragment will print result of measurement using all modes on all sources
 *  // to the standard output.
 *  DigitalIOMeasurementControl iomc(getDevicePointerFromSomewhere());
 *  if( iomc.digitalIOMeasurementMode.isValid() && iomc.digitalIOMeasurementSource.isValid() )
 *  {
 *    vector<pair<string, TDigitalIOMeasurementMode> > modeDict;
 *    iomc.digitalIOMeasurementMode.getTranslationDict( modeDict );
 *    const unsigned int modeCnt = iomc.digitalIOMeasurementMode.dictSize();
 *    vector<pair<string, TDigitalIOMeasurementSource> > srcDict;
 *    iomc.digitalIOMeasurementSource.getTranslationDict( srcDict );
 *    const unsigned int srcCnt = iomc.digitalIOMeasurementSource.dictSize();
 *    for( unsigned int i=0; i<modeCnt; i++ )
 *    {
 *      iomc.digitalIOMeasurementMode.write( modeDict[i].second );
 *      for( unsigned int j=0; j<srcCnt; j++ )
 *      {
 *        iomc.digitalIOMeasurementSource.write( srcDict[j].second );
 *        printf( "Digital I/O measurement result using mode '%s' at source '%s': %s\n", iomc.digitalIOMeasurementMode.readS().c_str(), iomc.digitalIOMeasurementSource.readS().c_str(), iomc.digitalIOMeasurementResult.readS().c_str() );
 *      }
 *    }
 *  }
 * \endcode
 *  \endif
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class DigitalIOMeasurementControl : public ComponentCollection
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::DigitalIOMeasurementControl object.
    explicit DigitalIOMeasurementControl(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ): ComponentCollection( pDev ), digitalIOMeasurementMode(), digitalIOMeasurementSource(),
        digitalIOMeasurementResult()
    {
        DeviceComponentLocator locator( pDev, dltIOSubSystem );
        HLIST hList = locator.findComponent( "DigitalIOMeasurementControl" );
        if( hList != INVALID_ID )
        {
            locator.bindSearchBase( locator.searchbase_id(), "DigitalIOMeasurementControl" );
            m_hRoot = locator.searchbase_id();
            locator.bindComponent( digitalIOMeasurementMode, "DigitalIOMeasurementMode" );
            locator.bindComponent( digitalIOMeasurementSource, "DigitalIOMeasurementSource" );
            locator.bindComponent( digitalIOMeasurementResult, "DigitalIOMeasurementResult" );
        }
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the type of measurement to perform.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigitalIOMeasurementMode.
     */
    PropertyIDigitalIOMeasurementMode digitalIOMeasurementMode;
    /// \brief An enumerated integer property defining where the measurement shall be performed.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDigitalIOMeasurementMode.
     */
    PropertyIDigitalIOMeasurementSource digitalIOMeasurementSource;
    /// \brief A float property \b (read-only) that will contain the result of the measurement.
    /**
     *  Reading this property will automatically perform a new measurement.
     *
     *  \note
     *  Please note that the signal connected to the digital input must match the selected digital input
     *  threshold(see \b mvIMPACT::acquire::IOSubSystemBlueFOX::digitalInputThreshold) in order to
     *  obtain valid results.
     */
    PropertyF digitalIOMeasurementResult;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIDigitalIOMeasurementMode getDigitalIOMeasurementMode( void ) const
    {
        return digitalIOMeasurementMode;
    }
    PropertyIDigitalIOMeasurementSource getDigitalIOMeasurementSource( void ) const
    {
        return digitalIOMeasurementSource;
    }
    PropertyF getDigitalIOMeasurementResult( void ) const
    {
        return digitalIOMeasurementResult;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

//-----------------------------------------------------------------------------
/// \brief A base class that provides access to the most common settings for a device(\b Device specific interface layout only).
/**
 *  Use one of the class derived from this class to get access to all the available
 *  settings.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class FullSettingsBase
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::FullSettingsBase object.
    explicit FullSettingsBase(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& settingName = "Base" ) : imageProcessing( pDev, settingName ), imageDestination( pDev, settingName ), basedOn()
    {
        DeviceComponentLocator locator( pDev, dltSetting, settingName );
        locator.bindComponent( basedOn, "BasedOn" );
    }
    virtual ~FullSettingsBase() {}
    PYTHON_ONLY( %immutable; )
    /// \brief Image processing related properties.
    ImageProcessing imageProcessing;
    /// \brief Properties to define the result images format.
    ImageDestination imageDestination;
    /// \brief A string property \b (read-only) containing the name of the setting this setting is based on.
    PropertyS basedOn;
    PYTHON_ONLY( %mutable; )
#   ifdef DOTNET_ONLY_CODE
    ImageProcessing& getImageProcessing( void )
    {
        return imageProcessing;
    }
    ImageDestination& getImageDestination( void )
    {
        return imageDestination;
    }
    PropertyS getBasedOn( void ) const
    {
        return basedOn;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief This class provides access to general settings as well as to settings which are unique for the \b mvBlueFOX(\b Device specific interface layout only).
/**
 *  To see a small code example on how you can work with object of this class see
 *  the detailed description of the class \b mvIMPACT::acquire::FunctionInterface as well.
 *  Statements made for the use of the class \b mvIMPACT::acquire::FullSettingsBase apply for
 *  this class as well.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class SettingsBlueFOX : public FullSettingsBase
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::SettingsBlueFOX object.
    explicit SettingsBlueFOX(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& name = "Base" ) : FullSettingsBase( pDev, name ), cameraSetting( pDev, name ) {}
    PYTHON_ONLY( %immutable; )
    /// \brief Camera related settings.
    CameraSettingsBlueFOX cameraSetting;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    CameraSettingsBlueFOX& getCameraSetting( void )
    {
        return cameraSetting;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief This class provides access to general settings as well as to settings which are unique for frame grabber devices(\b Device specific interface layout only).
/**
 *  To see a small code example on how you can work with object of this class see
 *  the detailed description of the class \b mvIMPACT::acquire::FunctionInterface as well.
 *  Statements made for the use of the class \b mvIMPACT::acquire::FullSettingsBase apply for
 *  this class as well.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class SettingsFrameGrabber : public FullSettingsBase
//-----------------------------------------------------------------------------
{
public:
    /// brief Constructs a new \b mvIMPACT::acquire::SettingsFrameGrabber object.
    explicit SettingsFrameGrabber(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev,
        /// [in] The name of the driver internal setting to access with this instance.
        /// A list of valid setting names can be obtained by a call to
        /// \b mvIMPACT::acquire::FunctionInterface::getAvailableSettings, new
        /// settings can be created with the function
        /// \b mvIMPACT::acquire::FunctionInterface::createSetting
        const std::string& name = "Base" ) : FullSettingsBase( pDev, name ), cameraSetting( pDev, name ), connector( pDev, name ) {}
    PYTHON_ONLY( %immutable; )
    /// \brief Camera related settings.
    CameraSettingsFrameGrabber cameraSetting;
    /// \brief Input channel related properties.
    Connector connector;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    CameraSettingsFrameGrabber& getCameraSetting( void )
    {
        return cameraSetting;
    }
    Connector& getConnector( void )
    {
        return connector;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVGRABBER_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION
/// \brief \b deprecated. Use the class \b mvIMPACT::acquire::SystemSettings instead(\b Device specific interface layout only).
/**
 *  \deprecated
 *  This class has been declared \b deprecated and might not be available in future releases.
 *  All features of this class are now available in \b mvIMPACT::acquire::SystemSettings as well, so please use
 *  this class instead.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
#       ifndef WRAP_PYTHON
typedef SystemSettings MVIMPACT_DEPRECATED_CPP( SystemBlueCOUGAR );
#       endif // #  ifndef WRAP_PYTHON
#   endif // #ifndef IGNORE_MVBLUECOUGAR_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class containing \b mvBlueFOX specific settings to control the overall behaviour of the driver(\b Device specific interface layout only).
/**
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class SystemBlueFOX : public SystemBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::SystemBlueFOX object.
    explicit SystemBlueFOX(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : SystemBase( pDev ), transferSize(), footerMode(), footerCheckEnable(), powerMode()
    {
        DeviceComponentLocator locator( pDev, dltSystemSettings );
        locator.bindComponent( powerMode, "PowerMode" );
        locator.bindSearchBase( locator.searchbase_id(), "Camera" );
        locator.bindComponent( transferSize, "TransferSize" );
        locator.bindComponent( footerMode, "FooterMode" );
        locator.bindComponent( footerCheckEnable, "FooterCheckEnable" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief An enumerated integer property defining the block size of the image data blocks transferred from the device.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBlueFOXTransferSize.
     */
    PropertyIBlueFOXTransferSize transferSize;
    /// \brief An enumerated integer property defining the footer mode of the device.
    /**
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBlueFOXFooterMode.
     */
    PropertyIBlueFOXFooterMode footerMode;
    /// \brief An enumerated integer property allowing to switch on/off the check of the image footer.
    /**
     *  The image footer contains certain additional data as e.g. the exposure time as used by the image
     *  sensor. To debug transfer related problems it sometimes can be useful to disable all internal
     *  data consistency checks by the driver.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     */
    PropertyIBoolean footerCheckEnable;
    /// \brief An enumerated integer property defining the power mode of the device.
    /**
     *  \note
     *  This property requires at least firmware version 39! See \b mvIMPACT::acquire::Device::firmwareVersion to find
     *  out which firmware is currently running on the device or use \b mvIMPACT::acquire::Device::updateFirmware()
     *  to update the firmware to the state compiled into the driver.
     *
     *  \note
     *  Do \b NOT modify this property when images are acquired in parallel. Switching off or on the power
     *  for a device that is capturing images at the same time will result in incorrect images! Therefore
     *  always make sure to stop image acquisition \b BEFORE modifying this property.
     *
     *  Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TDevicePowerMode.
     */
    PropertyIDevicePowerMode powerMode;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyIBlueFOXTransferSize getTransferSize( void ) const
    {
        return transferSize;
    }
    PropertyIBlueFOXFooterMode getFooterMode( void ) const
    {
        return footerMode;
    }
    PropertyIDevicePowerMode getPowerMode( void ) const
    {
        return powerMode;
    }
    PropertyIBoolean getFooterCheckEnable( void ) const
    {
        return footerCheckEnable;
    }
#   endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVBLUEFOX_SPECIFIC_DOCUMENTATION

#   ifndef IGNORE_MVV4L2_SPECIFIC_DOCUMENTATION
//-----------------------------------------------------------------------------
/// \brief A class containing \b mvV4L2 specific settings to control the overall behaviour of the driver(\b Device specific interface layout only).
/**
 *  \note UNDER CONSTRUCTION! Subject to change.
 *
 *  \note This class will only be available if \b mvIMPACT::acquire::Device::interfaceLayout is set to
 *  \b mvIMPACT::acquire::dilDeviceSpecific before the device is opened.
 */
class SystemV4L2 : public SystemBase
//-----------------------------------------------------------------------------
{
public:
    /// \brief Constructs a new \b mvIMPACT::acquire::SystemV4L2 object.
    explicit SystemV4L2(
        /// [in] A pointer to a \b mvIMPACT::acquire::Device object obtained from a \b mvIMPACT::acquire::DeviceManager object.
        Device* pDev ) : SystemBase( pDev ), volume(), balance(), bass(), treble(), mute(), loudness()
    {
        DeviceComponentLocator locator( pDev, dltSystemSettings );
        locator.bindSearchBase( locator.searchbase_id(), "V4L-Audio" );
        locator.bindComponent( volume, "Volume" );
        locator.bindComponent( balance, "Balance" );
        locator.bindComponent( bass, "Bass" );
        locator.bindComponent( treble, "Treble" );
        locator.bindComponent( mute, "Mute" );
        locator.bindComponent( loudness, "Loudness" );
    }
    PYTHON_ONLY( %immutable; )
    /// \brief If V4L2 device supports audio, this integer property adjusts the audio volume.
    /**
     *  This property represents a V4L2-audio-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI volume;
    /// \brief If V4L2 device supports audio, this integer property adjusts the audio balance.
    /**
     *  This property represents a V4L2-audio-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI balance;
    /// \brief If V4L2 device supports audio, this integer property adjusts the audio bass.
    /**
     *  This property represents a V4L2-audio-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI bass;
    /// \brief If V4L2 device supports audio, this integer property adjusts the audio treble.
    /**
     *  This property represents a V4L2-audio-control ID.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     *  If supported, a minimum value, a maximum value and step width will be defined. Thus, invalid values may be tuned after writing to fit within the step size,
     *  values too large or too small will raise an exception.
     */
    PropertyI treble;
    /// \brief If V4L2 device supports audio, this enumerated integer property sets audio mute on/off
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean mute;
    /// \brief If V4L2 device supports audio, this enumerated integer property sets audio loudness on/off.
    /**
     *  This property represents a boolean-valued V4L2-control ID. Valid values for this property are defined by the enumeration \b mvIMPACT::acquire::TBoolean.
     *
     *  \note This property is not supported by every device.
     *  Therefore always call the function \b mvIMPACT::acquire::Component::isValid to check if this property is available or not.
     */
    PropertyIBoolean loudness;
    PYTHON_ONLY( %mutable; )
#       ifdef DOTNET_ONLY_CODE
    PropertyI getVolume( void ) const
    {
        return volume;
    }
    PropertyI getBalance( void ) const
    {
        return balance;
    }
    PropertyI getBass( void ) const
    {
        return bass;
    }
    PropertyI getTreble( void ) const
    {
        return treble;
    }
    PropertyIBoolean getMute( void ) const
    {
        return mute;
    }
    PropertyIBoolean getLoudness( void ) const
    {
        return loudness;
    }
#       endif // #ifdef DOTNET_ONLY_CODE
};
#   endif // #ifndef IGNORE_MVV4L2_SPECIFIC_DOCUMENTATION

/// @}

#endif // #ifndef IGNORE_MVDEVICE_SPECIFIC_INTERFACE_DOCUMENTATION

#ifndef WRAP_DOTNET
inline void ExceptionFactory::raiseException( const char* pFunctionName, int lineNumber, int errorCode, HOBJ objectHandle /* = INVALID_ID*/, const std::string& additionalInfo /*= ""*/ )
{
    std::ostringstream oss;
    oss << pFunctionName << " (line: " << lineNumber << ")";
    switch( errorCode )
    {
    case PROPHANDLING_NOT_A_LIST:
        throw ENotAList( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NOT_A_PROPERTY:
        throw ENotAProperty( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NOT_A_METHOD:
        throw ENotAMethod( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NO_READ_RIGHTS:
        throw ENoReadRights( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NO_WRITE_RIGHTS:
        throw ENoWriteRights( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NO_MODIFY_SIZE_RIGHTS:
        throw ENoModifySizeRights( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_INCOMPATIBLE_COMPONENTS:
        throw EIncompatibleComponents( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_NO_USER_ALLOCATED_MEMORY:
        throw ENoUserAllocatedMemory( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_UNSUPPORTED_PARAMETER:
        throw EUnsupportedParameter( oss.str() );
    case PROPHANDLING_SIZE_MISMATCH:
        throw ESizeMismatch( "Size mismatch during query of values for " + Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_IMPLEMENTATION_MISSING:
        throw EImplementationMissing( oss.str() );
    case PROPHANDLING_INVALID_PROP_VALUE:
        throw EInvalidValue( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_PROP_TRANSLATION_TABLE_CORRUPTED:
        throw ETranslationTableCorrupted( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_PROP_VAL_ID_OUT_OF_BOUNDS:
        throw EValIDOutOfBounds( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_PROP_TRANSLATION_TABLE_NOT_DEFINED:
        throw ETranslationTableNotDefined( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_INVALID_PROP_VALUE_TYPE:
        throw EInvalidValueType( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_PROP_VAL_TOO_LARGE:
        {
            Property p( objectHandle );
            throw EValTooLarge( additionalInfo, p.readS( plMaxValue ), p.name(), oss.str() );
        }
    case PROPHANDLING_PROP_VAL_TOO_SMALL:
        {
            Property p( objectHandle );
            throw EValTooSmall( additionalInfo, p.readS( plMinValue ), p.name(), oss.str() );
        }
    case PROPHANDLING_COMPONENT_NOT_FOUND:
        throw EComponentNotFound( additionalInfo, oss.str() );
    case PROPHANDLING_LIST_ID_INVALID:
        throw EInvalidListID( oss.str() );
    case PROPHANDLING_COMPONENT_ID_INVALID:
        throw EComponentIDInvalid( oss.str() );
    case PROPHANDLING_LIST_ENTRY_OCCUPIED:
        throw EListEntryOccupied( oss.str() );
    case PROPHANDLING_LIST_CANT_ACCESS_DATA:
        throw ECantAccessData( additionalInfo, oss.str() );
    case PROPHANDLING_METHOD_PTR_INVALID:
        throw EMethodPtrInvalid( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_METHOD_INVALID_PARAM_LIST:
        throw EInvalidParameterList( oss.str() );
    case PROPHANDLING_INVALID_INPUT_PARAMETER:
        throw EInvalidInputParameter( oss.str() );
    case PROPHANDLING_INPUT_BUFFER_TOO_SMALL:
        throw EInputBufferTooSmall( oss.str() );
    case PROPHANDLING_WRONG_PARAM_COUNT:
        throw EWrongParamCount( oss.str() );
    case PROPHANDLING_UNSUPPORTED_OPERATION:
        throw EUnsupportedOperation( oss.str() );
    case PROPHANDLING_CANT_SERIALIZE_DATA:
        throw ECantSerializeData( Component( objectHandle ).name(), oss.str() );
    case PROPHANDLING_INVALID_FILE_CONTENT:
        throw EInvalidFileContent( additionalInfo, oss.str() );
    case PROPHANDLING_CANT_ALLOCATE_LIST:
        throw ECantAllocateNewList( oss.str() );
    case PROPHANDLING_CANT_REGISTER_COMPONENT:
        throw ECantRegisterComponent( oss.str() );
    case PROPHANDLING_PROP_VALIDATION_FAILED:
        {
            Property p( objectHandle );
            throw EValidationFailed( p.name(), oss.str() );
        }
    // unsupported:
    //PROPHANDLING_ACCESSTOKEN_CREATION_FAILED          = -2011
    //PROPHANDLING_COMPONENT_HAS_OWNER_ALREADY          = -2023,
    //PROPHANDLING_COMPONENT_ALREADY_REGISTERED         = -2024,
    //PROPHANDLING_SWIG_ERROR                           = -2028,
    //PROPHANDLING_COMPONENT_NO_CALLBACK_REGISTERED     = -2030,
    case DMR_DEV_NOT_FOUND:
    case DMR_INIT_FAILED:
    case DMR_DRV_ALREADY_IN_USE:
    case DMR_DEV_CANNOT_OPEN:
    case DMR_NOT_INITIALIZED:
    case DMR_DRV_CANNOT_OPEN:
    case DMR_DEV_REQUEST_QUEUE_EMPTY:
    case DMR_DEV_REQUEST_CREATION_FAILED:
    case DMR_INVALID_PARAMETER:
    case DMR_EXPORTED_SYMBOL_NOT_FOUND:
    case DEV_UNKNOWN_ERROR:
    case DEV_HANDLE_INVALID:
    case DEV_INPUT_PARAM_INVALID:
    case DEV_WRONG_INPUT_PARAM_COUNT:
    case DEV_CREATE_SETTING_FAILED:
    case DEV_REQUEST_CANT_BE_UNLOCKED:
    case DEV_INVALID_REQUEST_NUMBER:
    case DEV_LOCKED_REQUEST_IN_QUEUE:
    case DEV_NO_FREE_REQUEST_AVAILABLE:
    case DEV_WAIT_FOR_REQUEST_FAILED:
    case DEV_UNSUPPORTED_PARAMETER:
    case DEV_INVALID_RTC_NUMBER:
    case DMR_INTERNAL_ERROR:
    case DMR_INPUT_BUFFER_TOO_SMALL:
    case DEV_INTERNAL_ERROR:
    case DMR_LIBRARY_NOT_FOUND:
    case DMR_FUNCTION_NOT_IMPLEMENTED:
    case DMR_FEATURE_NOT_AVAILABLE:
    case DMR_EXECUTION_PROHIBITED:
    case DMR_FILE_NOT_FOUND:
    case DMR_INVALID_LICENCE:
    case DEV_SENSOR_TYPE_ERROR:
    case DMR_CAMERA_DESCRIPTION_INVALID:
    case DMR_NEWER_LIBRARY_REQUIRED:
    case DEV_ACCESS_DENIED:
    case DMR_PRELOAD_CHECK_FAILED:
    case DMR_CAMERA_DESCRIPTION_INVALID_PARAMETER:
        throw EDeviceManager( additionalInfo, oss.str(), static_cast<TDMR_ERROR>( errorCode ) );
    default:
        throw ImpactAcquireException( "Unknown error(" + additionalInfo + ")", oss.str(), errorCode );
    }
}
#endif // #ifndef WRAP_DOTNET

} // namespace acquire
} // namespace mvIMPACT

// restore Borland compiler switch 'force enums to the size of integer'
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#   ifdef _WIN32
#       ifdef __BORLANDC__
#           pragma option pop
#       endif // #ifdef __BORLANDC__
#   endif // _WIN32
#endif // !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)

#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_PYTHON)
#   ifndef MVIMPACT_USE_NAMESPACES
using namespace mvIMPACT::acquire;
#   endif // #ifndef MVIMPACT_USE_NAMESPACES
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_PYTHON

#ifdef _MSC_VER // is Microsoft compiler?
// restore old warning level
#   pragma warning( pop )
#endif // #ifdef _MSC_VER

#endif // MVIMPACT_ACQUIRE_H_
