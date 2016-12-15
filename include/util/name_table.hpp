#ifndef OSRM_UTIL_NAME_TABLE_HPP
#define OSRM_UTIL_NAME_TABLE_HPP

#include "util/range_table.hpp"
#include "util/shared_memory_vector_wrapper.hpp"
#include "util/typedefs.hpp"

#include <string>

#include <boost/utility/string_ref.hpp>
using StringView = boost::string_ref;

namespace osrm
{
namespace util
{

// While this could, theoretically, hold any names in the fitting format,
// the NameTable allows access to a part of the Datafacade to allow
// processing based on name indices.
class NameTable
{
  private:
    // FIXME should this use shared memory
    util::RangeTable<16, false> m_name_table;
    ShM<char, false>::vector m_names_char_list;

  public:
    NameTable(const std::string &filename);

    // This class provides a limited view over all the string data we serialize out.
    // The following functions are a subset of what is available.
    // See the data facades for they provide full access to this serialized string data.
    // (at time of writing this: get{Name,Ref,Pronunciation,Destinations}ForID(name_id);)
    std::string GetNameForID(const NameID id) const;
    std::string GetRefForID(const NameID id) const;
    std::string GetPronunciationForID(const NameID id) const;

    // TODO: keep old ones around while refactoring this; remove later and use string view ones

    StringView GetNameForID2(const NameID id) const;
    StringView GetRefForID2(const NameID id) const;
    StringView GetPronunciationForID2(const NameID id) const;
};
} // namespace util
} // namespace osrm

#endif // OSRM_UTIL_NAME_TABLE_HPP
