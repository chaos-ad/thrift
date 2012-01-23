/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements. See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership. The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef _THRIFT_ERL_UTILS_H
#define _THRIFT_ERL_UTILS_H 1

#include <set>
#include <map>
#include <list>
#include <vector>
#include <limits>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <exception>

#include <boost/optional.hpp>
#include <boost/mpl/identity.hpp>

#include <protocol/TBinaryProtocol.h>
#include <transport/TBufferTransports.h>

#include <erl_nif.h>

/////////////////////////////////////////////////////////////////////////////

namespace apache { namespace thrift { namespace erl_helpers {



typedef std::vector<ERL_NIF_TERM> tuple_t;
typedef ERL_NIF_TERM (*api_fn_t)(ErlNifEnv * env, ERL_NIF_TERM term);

struct atom_t : std::string
{
    atom_t(std::string const& str) : std::string(str) {}
};

struct enomem : public std::runtime_error
{
    enomem() : runtime_error("enomem") {}
};

struct invalid_type : public std::runtime_error
{
    invalid_type() : runtime_error("invalid_type") {}
};

/////////////////////////////////////////////////////////////////////////////

template <typename T>
T read_value(ErlNifEnv* env, ERL_NIF_TERM term);

template <typename T>
boost::optional<T> read_optional(ErlNifEnv* env, ERL_NIF_TERM term);

/////////////////////////////////////////////////////////////////////////////

inline bool read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<bool>)
{
    atom_t atom = read_value<atom_t>(env, term);
    if (atom ==  "true") return true;
    if (atom == "false") return false;
    throw invalid_type();
};

inline int16_t read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<int16_t>)
{
    int32_t result;
    if (!enif_get_int(env, term, &result)) {
        throw invalid_type();
    }
    if (result > std::numeric_limits<int16_t>::max()) {
        throw invalid_type();
    }
    return result;
};

inline int32_t read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<int32_t>)
{
    int32_t result;
    if (!enif_get_int(env, term, &result)) {
        throw invalid_type();
    }
    return result;
}

inline int64_t read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<int64_t>)
{
    int64_t result;
    if (!enif_get_int64(env, term, &result)) {
        throw invalid_type();
    }
    return result;
}

inline double read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<double>)
{
    double result;
    if (!enif_get_double(env, term, &result)) {
        return read_value<int64_t>(env, term);
    }
    return result;
}

inline std::string read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<std::string>)
{
    // Try binary first
    ErlNifBinary binary;
    if (enif_inspect_binary(env, term, &binary)) {
        return std::string(reinterpret_cast<char*>(binary.data), binary.size);
    }

    // Then try a list
    unsigned int length = 0;
    if (enif_get_list_length(env, term, &length)) {
        std::vector<char> buf(length+1, 0);
        std::size_t sz = enif_get_string(env, term, buf.data(), buf.size(), ERL_NIF_LATIN1);
        if (sz != buf.size()) {
            throw invalid_type();
        }

        return std::string(buf.data());
    }

    throw invalid_type();
}

inline atom_t read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<atom_t>)
{
    unsigned int length = 0;
    if (!enif_get_atom_length(env, term, &length, ERL_NIF_LATIN1)) {
        throw invalid_type();
    }

    std::vector<char> buf(length+1, 0);
    std::size_t sz = enif_get_atom(env, term, buf.data(), buf.size(), ERL_NIF_LATIN1);
    if (sz != buf.size()) {
        throw invalid_type();
    }

    std::string result(buf.data());
    return atom_t(result);
}

inline tuple_t read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity<tuple_t>)
{
    int arity = 0;
    ERL_NIF_TERM const* result;
    if (!enif_get_tuple(env, term, &arity, &result)) {
        throw invalid_type();
    }

    return tuple_t(result, result+arity);
}

template <typename T>
inline std::set<T> read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity< std::set<T> >)
{
    std::set<T> result;
    ERL_NIF_TERM head;
    ERL_NIF_TERM tail = term;
    while(!enif_is_empty_list(env, tail))
    {
        if (!enif_get_list_cell(env, tail, &head, &tail))
        {
            throw invalid_type();
        }
        result.insert( read_value<T>(env, head) );
    }
    return result;
}

template <typename T>
inline std::list<T> read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity< std::list<T> >)
{
    std::list<T> result;
    ERL_NIF_TERM head;
    ERL_NIF_TERM tail = term;
    while(!enif_is_empty_list(env, tail))
    {
        if (!enif_get_list_cell(env, tail, &head, &tail))
        {
            throw invalid_type();
        }
        result.push_back( read_value<T>(env, head) );
    }
    return result;
}

template <typename T>
inline std::vector<T> read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity< std::vector<T> >)
{
    std::vector<T> result;
    ERL_NIF_TERM head;
    ERL_NIF_TERM tail = term;
    while(!enif_is_empty_list(env, tail))
    {
        if (!enif_get_list_cell(env, tail, &head, &tail))
        {
            throw invalid_type();
        }
        result.push_back( read_value<T>(env, head) );
    }
    return result;
}

template <typename K, typename V>
inline std::map<K, V> read_value(ErlNifEnv* env, ERL_NIF_TERM term, boost::mpl::identity< std::map<K, V> >)
{
    std::map<K, V> result;
    std::list<tuple_t> list = read_value< std::list<tuple_t> >(env, term);
    std::list<tuple_t>::const_iterator i, end = list.end();
    for( i = list.begin(); i != end; ++i ) {
        result.insert( std::make_pair(
            read_value<K>(env, i->at(0)),
            read_value<V>(env, i->at(1)) )
        );
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

inline ERL_NIF_TERM write_value(ErlNifEnv* env, bool value)
{
    if (value) {
        return enif_make_atom(env, "true");
    } else {
        return enif_make_atom(env, "false");
    }
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, int16_t value)
{
    return enif_make_int(env, value);
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, int32_t value)
{
    return enif_make_int(env, value);
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, int64_t value)
{
    return enif_make_long(env, value);
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, double value)
{
    return enif_make_double(env, value);
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, std::string const& value)
{
    ErlNifBinary binary;
    if (!enif_alloc_binary(value.size(), &binary)) {
        throw enomem();
    }
    std::copy(value.begin(), value.end(), binary.data);
    return enif_make_binary(env, &binary);
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, atom_t const& value)
{
    return enif_make_atom(env, value.c_str());
}

inline ERL_NIF_TERM write_value(ErlNifEnv* env, tuple_t const& value)
{
    return enif_make_tuple_from_array(env, value.data(), value.size());
}


template <class T>
inline ERL_NIF_TERM write_value(ErlNifEnv* env, std::set<T> const& value)
{
    std::vector<ERL_NIF_TERM> terms;
    typename std::set<T>::const_iterator i, end = value.end();
    for( i = value.begin(); i != end; ++i )
    {
        terms.push_back( write_value(env, *i) );
    }
    return enif_make_list_from_array(env, terms.data(), terms.size());
}

template <class T>
inline ERL_NIF_TERM write_value(ErlNifEnv* env, std::list<T> const& value)
{
    std::vector<ERL_NIF_TERM> terms;
    typename std::list<T>::const_iterator i, end = value.end();
    for( i = value.begin(); i != end; ++i )
    {
        terms.push_back( write_value(env, *i) );
    }
    return enif_make_list_from_array(env, terms.data(), terms.size());
}

template <class T>
inline ERL_NIF_TERM write_value(ErlNifEnv* env, std::vector<T> const& value)
{
    std::vector<ERL_NIF_TERM> terms;
    typename std::vector<T>::const_iterator i, end = value.end();
    for( i = value.begin(); i != end; ++i )
    {
        terms.push_back( write_value(env, *i) );
    }
    return enif_make_list_from_array(env, terms.data(), terms.size());
}

template <class K, class V>
inline ERL_NIF_TERM write_value(ErlNifEnv* env, std::map<K, V> const& value)
{
    std::vector<ERL_NIF_TERM> terms;
    typename std::map<K, V>::const_iterator i, end = value.end();
    for( i = value.begin(); i != end; ++i )
    {
        std::vector<ERL_NIF_TERM> tuple(2);
        tuple[0] = write_value(env, i->first);
        tuple[1] = write_value(env, i->second);
        terms.push_back( write_value(env, tuple) );
    }
    return enif_make_list_from_array(env, terms.data(), terms.size());
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

template<typename T, typename U, typename Arg>
inline void read_required_field(ErlNifEnv* env, ERL_NIF_TERM term, U * object, void (U::*f) (Arg))
{
    T value = read_value<T>(env, term);
    (object->*f)(value);
}

template<typename T, typename U, typename Arg>
inline void read_optional_field(ErlNifEnv* env, ERL_NIF_TERM term, U * object, void (U::*f) (Arg))
{
    if (boost::optional<T> value = read_optional<T>(env, term))
    {
        (object->*f)(*value);
    }
}

/////////////////////////////////////////////////////////////////////////////


template <typename T>
inline T read_value(ErlNifEnv* env, ERL_NIF_TERM term)
{
    return read_value(env, term, boost::mpl::identity<T>());
}

template <typename T>
inline boost::optional<T> read_optional(ErlNifEnv* env, ERL_NIF_TERM term)
{
    if (enif_is_atom(env, term))
    {
        atom_t atom = read_value<atom_t>(env, term);
        if (atom == "undefined")
        {
            return boost::optional<T>();
        }
    }
    boost::optional<T> result = read_value<T>(env, term);
    return result;
}

/////////////////////////////////////////////////////////////////////////////

inline void check_record(ErlNifEnv * env, ERL_NIF_TERM term, std::string atom)
{
    atom_t recordname = read_value<atom_t>(env, term);
    if ( recordname != atom )
    {
        std::stringstream out;
        out << "Invalid record: " << recordname << "; expected: " << atom << std::endl;
        throw std::runtime_error(out.str());
    }
}

/////////////////////////////////////////////////////////////////////////////

template <class T>
ERL_NIF_TERM pack(ErlNifEnv * env, ERL_NIF_TERM const term)
{
    T object = read_value<T>(env, term);

    using namespace apache::thrift::protocol;
    using namespace apache::thrift::transport;

    boost::shared_ptr<TMemoryBuffer> buffer(new TMemoryBuffer());
    boost::shared_ptr<TBinaryProtocol> proto(new TBinaryProtocol(buffer));

    object.write(proto.get());
    std::string data = buffer.get()->getBufferAsString();

    ErlNifBinary binary;
    if (!enif_alloc_binary(data.size(), &binary)) {
        throw enomem();
    }
    std::copy(data.begin(), data.end(), binary.data);
    return enif_make_binary(env, &binary);
}

template <class T>
ERL_NIF_TERM unpack(ErlNifEnv * env, ERL_NIF_TERM term)
{
    using namespace apache::thrift::transport;
    using namespace apache::thrift::protocol;

    ErlNifBinary binary;
    if (!enif_inspect_binary(env, term, &binary)) {
        throw invalid_type();
    }

    boost::shared_ptr<TMemoryBuffer> buffer(new TMemoryBuffer(binary.data, binary.size));
    boost::shared_ptr<TBinaryProtocol> proto(new TBinaryProtocol(buffer));
    T object;
    object.read(proto.get());

    return write_value(env, object);
}

/////////////////////////////////////////////////////////////////////////////

}}}

#endif // _THRIFT_ERL_UTILS_H
