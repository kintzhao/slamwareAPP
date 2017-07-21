#pragma once

#include <map>

namespace rpos { namespace system {

    template < class TKey >
    class GenericMap {
    public:
        GenericMap()
        {}

        GenericMap(const GenericMap& that)
        {
            *this = that;
        }
        ~GenericMap() {
            for (auto iter = innerDict_.begin(); iter != innerDict_.end(); iter++)
                delete iter->second;
        }

    public:
        GenericMap& operator=(const GenericMap& that)
        {
            innerDict_.clear();

            for (auto iter = that.innerDict_.begin(); iter != that.innerDict_.end(); iter++)
            {
                innerDict_[iter->first] = iter->second->clone();
            }

            return *this;
        }

    public:
        template < class T >
        void put(const TKey& key, const T& value)
        {
            auto iter = innerDict_.find(key);

            if (iter != innerDict_.end())
            {
                delete iter->second;
            }

            innerDict_[key] = new holder<T>(value);
        }

        template < class T >
        bool contains(const TKey& key)
        {
            return find_<T>(key) != innerDict_.end();
        }

        template < class T >
        bool tryGet(const TKey& key, T& output)
        {
            auto iter = find_<T>(key);

            if (iter == innerDict_.end())
                return false;

            output = *(T*)iter->second->pointer();
            return true;
        }

        template < class T >
        const T& get(const TKey& key)
        {
            auto iter = find_<T>(key)->second->pointer();
            assert(iter != innerDict_.end());
            return *(T*)iter->second->pointer();
        }

        void clear()
        {
            innerDict_.clear();
        }

        void remove(const TKey& key)
        {
            innerDict_.erase(key);
        }

    private:
        class abs_holder {
        public:
            abs_holder()
            {}

            virtual ~abs_holder()
            {}

        public:
            virtual void* pointer() = 0;
            virtual const std::type_info& datatype() = 0;
            virtual abs_holder* clone() = 0;
        };

        template < class T >
        class holder : public abs_holder {
        public:
            holder(const T& that)
                : value_(that)
            {}

            ~holder()
            {}

        public:
            virtual void* pointer() {
                return &value_;
            }

            virtual const std::type_info& datatype() {
                return typeid(T);
            }

            virtual abs_holder* clone() {
                return new holder<T>(value_);
            }

        private:
            T value_;
        };

    private:
        typedef std::map<TKey, abs_holder*> inner_dict_t;
        typedef typename std::map<TKey, abs_holder*>::iterator inner_dict_iter_t;

    private:
        template < class T >
        inner_dict_iter_t find_(const std::string& key)
        {
            auto iter = innerDict_.find(key);

            if (iter == innerDict_.end())
                return iter;

            if (typeid(T) != iter->second->datatype())
                return innerDict_.end();

            return iter;
        }

    private:
        inner_dict_t innerDict_;
    };

} }
