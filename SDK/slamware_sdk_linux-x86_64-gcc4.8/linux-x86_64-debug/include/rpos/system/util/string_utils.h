#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cwchar>

#define RPOS_STRING_NPOS        ((size_t)-1)

namespace rpos { namespace system { namespace util {

    /**
    * \brief Convert value to string representation
    */
    RPOS_CORE_API std::string to_string(int value);

    /**
    * \brief Convert value to string representation
    */
    RPOS_CORE_API std::string to_string(long value);

    /**
    * \brief Convert value to string representation
    */
    RPOS_CORE_API std::string to_string(long long value);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, int& v, int base = 10);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, long& v, int base = 10);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, long long& v, int base = 10);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, std::uint64_t& v, int base = 10);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, float& v);

    /**
    * \brief Parse value from string
    */
    RPOS_CORE_API bool try_parse(const std::string& s, double& v);

    /**
    * \brief Remove leading spaces (including space, tab char) from a string
    */
    RPOS_CORE_API std::string trim_left(const std::string& that);

    /**
    * \brief Remove tailing spaces from a string
    */
    RPOS_CORE_API std::string trim_right(const std::string& that);

    /**
    * \brief Remove leading and tailing spaces from a string
    */
    RPOS_CORE_API std::string trim(const std::string& that);

    /**
    * \brief Replace all string parts matches `match` with `replace` in string `s`
    */
    RPOS_CORE_API std::string replace(const std::string& s, const std::string& match, const std::string& replace);

    /**
    * \brief Replace all string parts matches `match` with `replace` in string `s`
    */
    RPOS_CORE_API std::string replace(const std::string& s, char match, const std::string& replace);

    /**
    * \brief Replace all string parts matches `match` with `replace` in string `s`
    */
    RPOS_CORE_API std::string replace(const std::string& s, char match, char replace);

    /**
    * \brief Remove all string parts matches `match` in string `s`
    */
    RPOS_CORE_API std::string remove(const std::string& s, const std::string& match);

    /**
    * \brief Remove all string parts matches `match` in string `s`
    */
    RPOS_CORE_API std::string remove(const std::string& s, char match);

    /**
    * \brief Check if string `that` is pure spaces
    */
    RPOS_CORE_API bool is_whitespace(const std::string& that);

    /**
    * \brief Check if string `text` starts with string `match`
    */
    RPOS_CORE_API bool starts_with(const std::string& text, const std::string& match);

    /**
    * \brief Check if string `text` starts with character `match`
    */
    RPOS_CORE_API bool starts_with(const std::string& text, char match);

    /**
    * \brief Check if string `text` ends with string `match`
    */
    RPOS_CORE_API bool ends_with(const std::string& text, const std::string& match);

    /**
    * \brief Check if string `text` ends with string `match`
    */
    RPOS_CORE_API bool ends_with(const std::string& text, char match);

    /**
    * \brief Check if string `text` constains string `match`
    */
    RPOS_CORE_API bool contains(const std::string& text, const std::string& match);

    /**
    * \brief Check if string `text` constains character `match`
    */
    RPOS_CORE_API bool contains(const std::string& text, char match);

    /**
    * \brief Lookup string `match` in `text`
    *
    * \return The position of the found string, string::npos for not found
    */
    RPOS_CORE_API std::string::size_type find(const std::string& text, const std::string& match, std::string::size_type start = 0);

    /**
    * \brief Lookup string `match` in `text`
    *
    * \return The position of the found string, string::npos for not found
    */
    RPOS_CORE_API std::string::size_type find(const std::string& text, char match, std::string::size_type start = 0);

    /**
    * \brief Split string with `splitter`
    */
    RPOS_CORE_API std::vector<std::string> split(const std::string& that, char splitter, size_t max_parts = 0, bool merge_continuous_splitters = false);

    /**
    * \brief Split string with `splitter`, and trim the splitted parts
    */
    RPOS_CORE_API std::vector<std::string> split_trim(const std::string& that, char splitter, size_t max_parts = 0, bool merge_continuous_splitters = false);

    /**
    * \brief Concat strings with connector
    */
    RPOS_CORE_API std::string join(const std::vector<std::string>& strings, const std::string& connector);

    /**
    * \brief Concat strings with connector
    */
    RPOS_CORE_API std::string join(const std::vector<std::string>& strings, char connector);

    /**
    * \brief Concat strings
    */
    RPOS_CORE_API std::string join(const std::vector<std::string>& strings);

    /**
    * \brief Repeat string
    *
    * For instance: string("hello") * 5 will produce a string "hellohellohellohellohello"
    */
    RPOS_CORE_API std::string operator*(const std::string& a, int b);

    /**
    * \brief Repeat string
    *
    * For instance: 5 * string("hello") will produce a string "hellohellohellohellohello"
    */
    RPOS_CORE_API std::string operator*(int a, const std::string& b);
    
    /**
    * \brief Transform string to its hex representation in textual format.  "ab" to "6162"
    */
    RPOS_CORE_API std::string stringToHexString(const std::string& s);

    /**
    * \brief Transform string's hex representation in textual format back to string.  "6162" to "ab"
    */
    RPOS_CORE_API std::string hexStringToString(const std::string& s);

    /**
    * \brief Convert String from UTF8 to WCS
    */
    RPOS_CORE_API bool utf8_to_wcs(::std::wstring& rDest, const char* pcSrc, size_t szLen);
    RPOS_CORE_API bool utf8_to_wcs(::std::wstring& rDest, const ::std::string& rcSrc);
    inline bool utf8_to_wcs(::std::wstring& rDest, const char* pcSrc)
    {
        return utf8_to_wcs(rDest, pcSrc, ::strlen(pcSrc));
    }
    inline ::std::wstring utf8_to_wcs(const ::std::string& rcSrc)
    {
        ::std::wstring wstrRet;
        utf8_to_wcs(wstrRet, rcSrc);
        return wstrRet;
    }

    /**
    * \brief Convert String from WCS to UTF8
    */
    RPOS_CORE_API bool wcs_to_utf8(::std::string& rDest, const wchar_t* pcSrc, size_t szLen);
    RPOS_CORE_API bool wcs_to_utf8(::std::string& rDest, const ::std::wstring& rcSrc);
    inline bool wcs_to_utf8(::std::string& rDest, const wchar_t* pcSrc)
    {
        return wcs_to_utf8(rDest, pcSrc, ::wcslen(pcSrc));
    }
    inline ::std::string wcs_to_utf8(const ::std::wstring& rcSrc)
    {
        ::std::string strRet;
        wcs_to_utf8(strRet, rcSrc);
        return strRet;
    }

    /**
    * \brief Convert String from ACP to WCS
    */
    RPOS_CORE_API bool acp_to_wcs(::std::wstring& rDest, const char* pcSrc, size_t szLen);
    inline bool acp_to_wcs(::std::wstring& rDest, const char* pcSrc)
    {
        return acp_to_wcs(rDest, pcSrc, ::strlen(pcSrc));
    }
    inline bool acp_to_wcs(::std::wstring& rDest, const ::std::string& rcSrc)
    {
        return acp_to_wcs(rDest, rcSrc.c_str(), rcSrc.length());
    }
    inline ::std::wstring acp_to_wcs(const ::std::string& rcSrc)
    {
        ::std::wstring wstrRet;
        acp_to_wcs(wstrRet, rcSrc);
        return wstrRet;
    }

    /**
    * \brief Convert String from WCS to ACP
    */
    RPOS_CORE_API bool wcs_to_acp(::std::string& rDest, const wchar_t* pcSrc, size_t szLen);
    inline bool wcs_to_acp(::std::string& rDest, const wchar_t* pcSrc)
    {
        return wcs_to_acp(rDest, pcSrc, ::wcslen(pcSrc));
    }
    inline bool wcs_to_acp(::std::string& rDest, const ::std::wstring& rcSrc)
    {
        return wcs_to_acp(rDest, rcSrc.c_str(), rcSrc.length());
    }
    inline ::std::string wcs_to_acp(const ::std::wstring& rcSrc)
    {
        ::std::string strRet;
        wcs_to_acp(strRet, rcSrc);
        return strRet;
    }

} } }
