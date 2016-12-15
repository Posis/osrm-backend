#include "util/name_table.hpp"
#include "storage/io.hpp"
#include "util/exception.hpp"
#include "util/log.hpp"

#include <algorithm>
#include <iterator>
#include <limits>

#include <boost/filesystem/fstream.hpp>

namespace osrm
{
namespace util
{

NameTable::NameTable(const std::string &filename)
{
    storage::io::FileReader name_stream_file_reader(filename,
                                                    storage::io::FileReader::HasNoFingerprint);

    m_name_table.ReadARangeTable(name_stream_file_reader);

    const auto number_of_chars = name_stream_file_reader.ReadElementCount32();

    m_names_char_list.resize(number_of_chars + 1); //+1 gives sentinel element
    m_names_char_list.back() = 0;
    if (number_of_chars > 0)
    {
        name_stream_file_reader.ReadInto(&m_names_char_list[0], number_of_chars);
    }
    else
    {
        util::Log() << "list of street names is empty in construction of name table from: \""
                    << filename << "\"";
    }
}

std::string NameTable::GetNameForID(const NameID id) const
{
    if (std::numeric_limits<NameID>::max() == id)
    {
        return "";
    }
    auto range = m_name_table.GetRange(id);

    std::string result;
    result.reserve(range.size());
    if (range.begin() != range.end())
    {
        result.resize(range.back() - range.front() + 1);
        std::copy(m_names_char_list.begin() + range.front(),
                  m_names_char_list.begin() + range.back() + 1,
                  result.begin());
    }
    return result;
}

std::string NameTable::GetRefForID(const NameID id) const
{
    // Way string data is stored in blocks based on `id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                                      ^     ^
    //                                      [range)
    //                                       ^ id + 3
    //
    // `id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_REF = 3u;
    return GetNameForID(id + OFFSET_REF);
}

std::string NameTable::GetPronunciationForID(const NameID id) const
{
    // Way string data is stored in blocks based on `id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                      ^               ^
    //                      [range)
    //                       ^ id + 2
    //
    // `id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_PRONUNCIATION = 2u;
    return GetNameForID(id + OFFSET_PRONUNCIATION);
}

StringView NameTable::GetNameForID2(const NameID id) const
{
    if (std::numeric_limits<NameID>::max() == id)
    {
        return {};
    }

    auto range = m_name_table.GetRange(id);

    if (range.begin() == range.end())
    {
        return {};
    }

    auto first = begin(m_names_char_list) + range.front();
    auto last = begin(m_names_char_list) + range.back() + 1;
    auto len = last - first;

    return StringView{&*first, len};
}

StringView NameTable::GetRefForID2(const NameID id) const
{
    // Way string data is stored in blocks based on `id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                                      ^     ^
    //                                      [range)
    //                                       ^ id + 3
    //
    // `id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_REF = 3u;
    return GetNameForID(id + OFFSET_REF);
}

StringView NameTable::GetPronunciationForID2(const NameID id) const
{
    // Way string data is stored in blocks based on `id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                      ^               ^
    //                      [range)
    //                       ^ id + 2
    //
    // `id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_PRONUNCIATION = 2u;
    return GetNameForID(id + OFFSET_PRONUNCIATION);
}

} // namespace util
} // namespace osrm
