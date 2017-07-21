#pragma once

#include <rpos/core/rpos_core_config.h>
#include <string>
#include <vector>

namespace rpos { namespace system { namespace util {

    /**
    * \brief Check if a file exists
    */
    RPOS_CORE_API bool file_exists(const std::string& filename);

    /**
    * \brief Check if a directory exists
    */
    RPOS_CORE_API bool dir_exists(const std::string& filename);

    /**
    * \brief Create a directory
    */
    RPOS_CORE_API bool dir_create(const std::string& path, bool createParentDirectoryIfNotExist = true);

    /**
    * \brief Copy file
    */
    RPOS_CORE_API bool file_copy(const std::string& from, const std::string& to);

    /**
    * \brief Move file
    */
    RPOS_CORE_API bool file_move(const std::string& from, const std::string& to);

    /**
    * \brief Delete file
    */
    RPOS_CORE_API bool file_del(const std::string& path);

    /**
    * \brief Get temp directory
    */
    RPOS_CORE_API std::string temp_dir();

    /**
    * \brief Get the size of particular file
    */
    RPOS_CORE_API size_t file_size(const std::string& filename);

    /**
    * \brief Read all content of the file
    */
    RPOS_CORE_API std::string read_all_text(const std::string& filename);

    /**
    * \brief Read all lines of a file
    */
    RPOS_CORE_API std::vector<std::string> read_all_lines(const std::string& filename);

    /**
    * \brief Write text to a file
    */
    RPOS_CORE_API bool write_all_text(const std::string& filename, const std::string& content);

    /**
    * \brief Read all bytes of the file
    */
    RPOS_CORE_API std::vector<unsigned char> read_all_bytes(const std::string& filename);

    /**
    * \brief Write all bytes to the file
    */
    RPOS_CORE_API bool write_all_bytes(const std::string& filename, const std::vector<unsigned char>& buffer);

    /**
    * \brief Write all bytes to the file
    */
    RPOS_CORE_API bool write_all_bytes(const std::string& filename, const unsigned char* buffer, size_t size);

} } }
