#include <iostream>
#include <vector>
#include <map>
#include <string>

#ifndef PRETTYPRINT_H
#define PRETTYPRINT_H

// Handy functions for printing vectors and pairs
template <typename T>
std::string PrettyPrint(T& toprint, bool add_delimiters=false)
{
    std::ostringstream strm;
    strm << toprint;
    return strm.str();
}

template <typename T>
std::string PrettyPrint(std::vector<T>& vectoprint, bool add_delimiters=false)
{
    std::ostringstream strm;
    if (vectoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "[" << PrettyPrint(vectoprint[0], add_delimiters);
            for (size_t idx = 1; idx < vectoprint.size(); idx++)
            {
                strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
            }
            strm << "]";
        }
        else
        {
            strm << PrettyPrint(vectoprint[0], add_delimiters);
            for (size_t idx = 1; idx < vectoprint.size(); idx++)
            {
                strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
            }
        }
    }
    return strm.str();
}

template <typename A, typename B>
std::string PrettyPrint(std::pair<A, B>& pairtoprint, bool add_delimiters=false)
{
    std::ostringstream strm;
    if (add_delimiters)
    {
        strm << "<" << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters) << ">";
    }
    else
    {
        strm << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters);
    }
    return strm.str();
}

template <typename A, typename B>
std::string PrettyPrint(std::map<A, B>& maptoprint, bool add_delimiters=false)
{
    std::ostringstream strm;
    if (maptoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "{";
            typename std::map<A, B>::iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
            strm << "}";
        }
        else
        {
            typename std::map<A, B>::iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
        }
    }
    return strm.str();
}

#endif // PRETTYPRINT_H
