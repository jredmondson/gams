/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/

/**
 * @file ReferenceFrame.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#include <gams/pose/ReferenceFrameFwd.h>
#include <gams/pose/ReferenceFrame.h>
#include <gams/pose/CartesianFrame.h>
#include <gams/pose/GPSFrame.h>
#include <gams/pose/Linear.h>
#include <gams/pose/Angular.h>
#include <gams/pose/Quaternion.h>

#include <random>

using madara::knowledge::KnowledgeBase;
using madara::knowledge::KnowledgeRecord;
using madara::knowledge::ContextGuard;
using madara::knowledge::KnowledgeMap;
using madara::knowledge::containers::NativeDoubleVector;
using kmiter = KnowledgeMap::iterator;
using kmpair = KnowledgeMap::value_type;

namespace gams
{
  namespace pose
  {
    std::map<std::string, std::weak_ptr<ReferenceFrameIdentity>>
      ReferenceFrameIdentity::idents_;

    uint64_t ReferenceFrameIdentity::default_expiry_ = -1;

    std::mutex ReferenceFrameIdentity::idents_lock_;

    std::shared_ptr<ReferenceFrameIdentity>
      ReferenceFrameIdentity::find(std::string id)
    {
      std::lock_guard<std::mutex> guard(idents_lock_);

      auto find = idents_.find(id);
      if (find != idents_.end()) {
        auto ret = find->second.lock();
        return ret;
      }
      return nullptr;
    }

    void ReferenceFrameIdentity::gc()
    {
      std::lock_guard<std::mutex> guard(idents_lock_);

      for (auto ident_iter = idents_.begin(); ident_iter != idents_.end();) {
        if (auto ident = ident_iter->second.lock()) {
          std::lock_guard<std::mutex> guard(ident->versions_lock_);

          for (auto ver_iter = ident->versions_.begin(); ver_iter != ident->versions_.end();) {
            if (ver_iter->second.expired()) {
              auto tmp = ver_iter;
              ++ver_iter;
              ident->versions_.erase(tmp);
            } else {
              ++ver_iter;
            }
          }
          ++ident_iter;
        } else {
          auto tmp = ident_iter;
          ++ident_iter;
          idents_.erase(tmp);
        }
      }
    }

    std::shared_ptr<ReferenceFrameIdentity>
      ReferenceFrameIdentity::lookup(std::string id)
    {
      std::lock_guard<std::mutex> guard(idents_lock_);

      auto find = idents_.find(id);
      if (find != idents_.end()) {
        auto ret = find->second.lock();
        if (ret) {
          return ret;
        }
      }
      auto val = std::make_shared<ReferenceFrameIdentity>(id, default_expiry_);
      std::weak_ptr<ReferenceFrameIdentity> weak{val};
      if (find != idents_.end()) {
        find->second = std::move(weak);
      } else {
        idents_.insert(std::make_pair(std::move(id), std::move(weak)));
      }
      return val;
    }

    static std::string make_random_id(size_t len)
    {
      // Avoid letters/numbers easily confused with others
      static const char alphabet[] = "23456789CDFHJKMNPRSTWXY";

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, sizeof(alphabet) - 2);

      std::string ret;
      ret.reserve(len + 2);
      ret += "{";
      for (size_t i = 0; i < len; ++i) {
        ret += alphabet[dis(gen)];
      }
      ret += "}";
      return ret;
    }

    std::shared_ptr<ReferenceFrameIdentity>
      ReferenceFrameIdentity::make_guid()
    {
      std::string key;
      decltype(idents_)::iterator find;
      for (;;) {
        key = make_random_id(30); // Over 128 bits of randomness

        std::lock_guard<std::mutex> guard(idents_lock_);
        find = idents_.find(key);
        if (find != idents_.end()) {
          if (!find->second.expired()) {
            continue;
          }
        }

        auto val = std::make_shared<ReferenceFrameIdentity>(key, default_expiry_);

        std::weak_ptr<ReferenceFrameIdentity> weak{val};
        if (find != idents_.end()) {
          find->second = std::move(weak);
        } else {
          idents_.insert(std::make_pair(std::move(key), std::move(weak)));
        }
        return val;
      }
    }

    namespace {
      int compare_prefix(const std::string &s,
          const char *prefix, size_t prefix_size)
      {
        return (s.compare(0, prefix_size, prefix, prefix_size));
      }

      int compare_suffix(const std::string &s,
          const char *suffix, size_t suffix_size)
      {
        return (s.compare(s.size() - suffix_size, suffix_size,
              suffix, suffix_size));
      }

      template<typename Iter>
      std::reverse_iterator<Iter> rev(Iter i) {
        return std::reverse_iterator<Iter>(i);
      }

      template<typename Iter>
      Iter rev(std::reverse_iterator<Iter> i) {
        return i.base();
      }

      std::pair<kmiter, kmiter> get_range(KnowledgeMap &map,
          std::string low, std::string high)
      {
        auto iter_low = rev(map.lower_bound(low));
        auto iter_high = rev(map.upper_bound(high));

        auto lambda_for = [](const std::string &s) {
          return [&s](const kmpair &p){
              return compare_prefix(p.first, s.c_str(), s.size()) < 0; };
        };

        iter_low = std::find_if(iter_low, map.rend(), lambda_for(low));
        iter_high = std::find_if(iter_high, map.rend(), lambda_for(high));

        return {rev(iter_low), rev(iter_high)};
      }
    }

    void ReferenceFrameIdentity::expire_older_than(
        KnowledgeBase &kb,
        uint64_t time,
        std::string prefix) const
    {
      impl::make_kb_key(prefix, id_);
      std::string key_low = std::move(prefix);
      std::string key_high = key_low;
      impl::make_kb_key(key_low, 0UL);
      impl::make_kb_key(key_high, time);

      {
        ContextGuard guard(kb);

        KnowledgeMap &map = kb.get_context().get_map_unsafe();
        auto range = get_range(map, std::move(key_low), std::move(key_high));
        map.erase(range.first, range.second);
      }

      {
        std::lock_guard<std::mutex> guard(versions_lock_);

        auto iter = versions_.begin();
        while (iter != versions_.end()) {
          auto cur = iter;
          ++iter;
          if (cur->first >= time) {
            break;
          }
          versions_.erase(cur);
        }
      }
    }

    void ReferenceFrameVersion::save_as(
            KnowledgeBase &kb,
            std::string key,
            uint64_t expiry,
            const std::string &prefix) const
    {
      key += ".";
      size_t pos = key.size();

      {
        ContextGuard guard(kb);

        if (type() != Cartesian) {
          key += "type";
          kb.set(key, name());
          key.resize(pos);
        }

        const ReferenceFrame &parent = origin_frame();
        if (parent.valid()) {
          key += "parent";
          kb.set(key, parent.id());
          key.resize(pos);
        }

        key += "origin";
        NativeDoubleVector vec(key, kb, 6);
        origin().to_container(vec);

        interpolated_ = false;
        ident().register_version(timestamp(),
            const_cast<ReferenceFrameVersion*>(this)->shared_from_this());
      }

      if (timestamp() > expiry) {
        uint64_t cap;
        if (timestamp() == (uint64_t)-1) {
          cap = timestamp() - 1;
        } else {
          cap = timestamp() - expiry;
        }
        ident().expire_older_than(kb, cap, prefix);
      }
    }

    namespace {
      std::pair<std::shared_ptr<ReferenceFrameVersion>, std::string>
        load_single( KnowledgeBase &kb, const std::string &id,
            uint64_t timestamp, std::string prefix)
      {
        auto ident = ReferenceFrameIdentity::find(id);
        if (ident) {
          auto ver = ident->get_version(timestamp);
          if (ver) {
            //std::cerr << id << " already loaded" << std::endl;
            return std::make_pair(std::move(ver), std::string());
          }
        }

        auto key = std::move(prefix);
        impl::make_kb_key(key, id, timestamp);

        key += ".";
        size_t pos = key.size();

        ContextGuard guard(kb);
        KnowledgeMap &map = kb.get_context().get_map_unsafe();

        std::string parent_name;

        key += "parent";
        auto find = map.find(key);
        if (find != map.end()) {
          parent_name = find->second.to_string();
        }
        key.resize(pos);

        const ReferenceFrameType *type = Cartesian;
        key += "type";
        find = map.find(key);
        //std::cerr << "Looking for single " << key << std::endl;
        if (find != map.end()) {
          std::string type_name = find->second.to_string();
          //std::cerr << "loaded " << type_name << " frame: " << id << std::endl;
          if (type_name == "GPS") {
            type = GPS;
          }
        }
        key.resize(pos);

        Pose origin(ReferenceFrame{});
        key += "origin";
        find = map.find(key);
        if (find != map.end()) {
          NativeDoubleVector vec(key, kb, 6);
          origin.from_container(vec);
        } else {
          return std::make_pair(std::shared_ptr<ReferenceFrameVersion>(),
              std::string());
        }

        auto ret = std::make_shared<ReferenceFrameVersion>(
            type, id, std::move(origin), timestamp);

        //std::cerr << "Made new " << id << " at " << (void*)ret.get() << std::endl;

        return std::make_pair(std::move(ret), std::move(parent_name));
      }
    }

    ReferenceFrame ReferenceFrameVersion::load_exact(
          KnowledgeBase &kb,
          const std::string &id,
          uint64_t timestamp,
          std::string prefix)
    {
      ContextGuard guard(kb);

      auto ret = load_single(kb, id, timestamp, prefix);
      if (!ret.first) {
        return ReferenceFrame();
      }
      if (ret.second.size() > 0) {
        auto parent = load(kb, ret.second, timestamp, std::move(prefix));
        if (!parent.valid()) {
          //std::cerr << "Couldn't find " << ret.second << std::endl;
          return ReferenceFrame();
        }
        ret.first->mut_origin().frame(parent);
      }

      ret.first->ident().register_version(timestamp, ret.first);
      return ReferenceFrame(ret.first);
    }

    namespace {
      uint64_t timestamp_from_key(const char *key)
      {
        char *end;
        uint64_t ret = strtoull(key, &end, 16);
        if (key == end) {
          return -1;
        }
        return ret;
      }

      std::pair<uint64_t, uint64_t> find_nearest_neighbors(
          KnowledgeBase &kb, const std::string &id,
          uint64_t timestamp, std::string prefix)
      {
        static const char suffix[] = ".origin";
        static const size_t suffix_len = sizeof(suffix) - 1;

        auto key = std::move(prefix);

        impl::make_kb_key(key, id);

        size_t len = key.size();

        impl::make_kb_key(key, timestamp);

        key += suffix;

        ContextGuard guard(kb);
        KnowledgeMap &map = kb.get_context().get_map_unsafe();

        auto find = map.lower_bound(key);
        auto next = find;

        //std::cerr << "Looking for neighbors " << key << std::endl;

        if (next->first == key) {
          //std::cerr << "Found it exactly." << std::endl;
          return std::make_pair(timestamp, timestamp);
        }

        while (next != map.end()) {
          const std::string &cur = next->first;
          if (compare_prefix(cur, key.c_str(), len) != 0) {
            next = map.end();
          } else if (compare_suffix(cur, suffix, suffix_len) == 0) {
            break;
          } else {
            ++next;
          }
        }

        auto prev = find;

        if (prev != map.begin()) {
          --prev;
        }

        while (prev != map.end()) {
          const std::string &cur = prev->first;
          if (compare_prefix(cur, key.c_str(), len) != 0) {
            prev = map.end();
          } else if (compare_suffix(cur, suffix, suffix_len) == 0) {
            break;
          } else if (prev == map.begin()) {
            prev = map.end();
          } else {
            --prev;
          }
        }

        uint64_t next_time = -1;
        if (next != map.end()) {
          next_time = timestamp_from_key(&next->first[len + 1]);
        }

        uint64_t prev_time = -1;
        if (prev != map.end()) {
          prev_time = timestamp_from_key(&prev->first[len + 1]);
        }

        return std::make_pair(prev_time, next_time);
      }
    }

    uint64_t ReferenceFrameVersion::latest_timestamp(
            madara::knowledge::KnowledgeBase &kb,
            const std::string &id,
            std::string prefix)
    {
      ContextGuard guard(kb);

      auto ret = find_nearest_neighbors(kb, id, -1, prefix).first;
      //std::cerr << "Latest for " << id << " is " << ret << std::endl;

      auto key = std::move(prefix);
      impl::make_kb_key(key, id, ret);
      key += ".parent";
      KnowledgeMap &map = kb.get_context().get_map_unsafe();
      auto find = map.find(key);
      if (find != map.end()) {
        auto p = latest_timestamp(kb, find->second.to_string());
        if (p < ret) {
          return p;
        }
      }
      return ret;
    }

    ReferenceFrame ReferenceFrameVersion::load(
            KnowledgeBase &kb,
            const std::string &id,
            uint64_t timestamp,
            std::string prefix)
    {
      ContextGuard guard(kb);

      if (timestamp == (uint64_t)-1) {
        timestamp = latest_timestamp(kb, id, prefix);
      }

      ReferenceFrame ret = load_exact(kb, id, timestamp, prefix);
      if (ret.valid()) {
        return ret;
      }

      ret = load_exact(kb, id, -1, prefix);
      if (ret.valid()) {
        return ret;
      }

      auto pair = find_nearest_neighbors(kb, id, timestamp, prefix);

      //std::cerr << "Nearest " << id << " " << pair.first << " " << timestamp << " " << pair.second << std::endl;

      if (pair.first == (uint64_t)-1 || pair.second == (uint64_t)-1) {
        //std::cerr << "No valid timestamp pair for " << id << std::endl;
        return {};
      }

      auto prev = load_single(kb, id, pair.first, prefix);
      auto next = load_single(kb, id, pair.second, prefix);

      ReferenceFrame parent;

      if (prev.first && next.first) {
        const std::string &parent_id = prev.second;
        const std::string &next_parent_id = next.second;

        if (parent_id != next_parent_id) {
          //std::cerr << "Mismatched frame parents " << parent_id << "  " << next_parent_id << std::endl;
          return {};
        }

        if (parent_id != "") {
          //std::cerr << "Loading " << id << "'s parent " << parent_id << std::endl;
          parent = load(kb, parent_id, timestamp, prefix);

          //std::cerr << "Interpolating " << id << " with parent " <<
            //(parent.valid() ? parent.id() : "INVALID") << std::endl;
        } else {
          //std::cerr << "Frame " << id << " has no parent" << std::endl;
        }
        return prev.first->interpolate(next.first, std::move(parent), timestamp);
      }

      //std::cerr << "No interpolation found for " << id << std::endl;
      return {};
    }

    // TODO implement O(height) algo instead of O(height ^ 2)
    const ReferenceFrame *find_common_frame(
      const ReferenceFrame *from, const ReferenceFrame *to,
      std::vector<const ReferenceFrame *> *to_stack)
    {
      const ReferenceFrame *cur_to = to;
      for(;;)
      {
        const ReferenceFrame *cur_from = from;
        for(;;)
        {
          if(*cur_to == *cur_from)
          {
            return cur_to;
          }
          cur_from = &cur_from->origin().frame();
          if(!cur_from->valid())
            break;
        }
        if(to_stack)
          to_stack->push_back(cur_to);
        cur_to = &cur_to->origin().frame();
        if(!cur_to->valid())
          break;
      }
      return nullptr;
    }

    namespace simple_rotate {
      void orient_linear_vec(
            double &x, double &y, double &z,
            double rx, double ry, double rz,
            bool reverse)
      {
        if(rx == 0 && ry == 0 && rz == 0)
          return;

        Quaternion locq(x, y, z, 0);
        Quaternion rotq(rx, ry, rz);

        if(reverse)
          rotq.conjugate();

        locq.orient_by(rotq);
        locq.to_linear_vector(x, y, z);
      }

      void transform_angular_to_origin(
                        const ReferenceFrameType *,
                        const ReferenceFrameType *,
                        double orx, double ory, double orz,
                        double &rx, double &ry, double &rz)
      {
        Quaternion in_quat(rx, ry, rz);
        Quaternion origin_quat(orx, ory, orz);
        in_quat *= origin_quat;
        in_quat.to_angular_vector(rx, ry, rz);
      }

      void transform_angular_from_origin(
                        const ReferenceFrameType *,
                        const ReferenceFrameType *,
                        double orx, double ory, double orz,
                        double &rx, double &ry, double &rz)
      {
        Quaternion in_quat(rx, ry, rz);
        Quaternion origin_quat(orx, ory, orz);
        origin_quat.conjugate();
        in_quat *= origin_quat;
        in_quat.to_angular_vector(rx, ry, rz);
      }

      void transform_pose_to_origin(
                      const ReferenceFrameType *other,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz)
      {
        self->transform_linear_to_origin(other, self,
                          ox, oy, oz,
                          orx, ory,
                          orz, x, y, z);
        self->transform_angular_to_origin(other, self,
                          orx, ory, orz,
                          rx, ry, rz);
      }

      void transform_pose_from_origin(
                      const ReferenceFrameType *other,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz)
      {
        self->transform_linear_from_origin(other, self,
                          ox, oy, oz,
                          orx, ory,
                          orz, x, y, z);
        self->transform_angular_from_origin(other, self,
                          orx, ory, orz,
                          rx, ry, rz);
      }

      double calc_angle(
                  const ReferenceFrameType *,
                  double rx1, double ry1, double rz1,
                  double rx2, double ry2, double rz2)
      {
        Quaternion quat1(rx1, ry1, rz1);
        Quaternion quat2(rx2, ry2, rz2);
        return quat1.angle_to(quat2);
      }
    }

    const ReferenceFrame &default_frame (void)
    {
      static ReferenceFrame frame{"default_frame", Pose(ReferenceFrame(), 0, 0)};
      return frame;
    }
  }
}
