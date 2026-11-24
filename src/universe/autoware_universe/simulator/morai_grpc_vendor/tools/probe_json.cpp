// 신규 — 26.09.24 _ YSH _ probe_json.hpp 구현 (직렬화만, 파싱 없음)
//
// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "probe_json.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

namespace morai_grpc_vendor::probe
{

Json::Json(bool v) : type_(Type::kBool), b_(v)
{
}
Json::Json(int32_t v) : type_(Type::kInt), i_(v)
{
}
Json::Json(int64_t v) : type_(Type::kInt), i_(v)
{
}
Json::Json(uint32_t v) : type_(Type::kUint), u_(v)
{
}
Json::Json(uint64_t v) : type_(Type::kUint), u_(v)
{
}
Json::Json(double v) : type_(Type::kDouble), d_(v)
{
}
Json::Json(const char * v) : type_(Type::kString), s_(v ? v : "")
{
}
Json::Json(std::string v) : type_(Type::kString), s_(std::move(v))
{
}

Json Json::object()
{
  Json j;
  j.type_ = Type::kObject;
  return j;
}

Json Json::array()
{
  Json j;
  j.type_ = Type::kArray;
  return j;
}

Json Json::raw(std::string json_text)
{
  Json j;
  j.type_ = Type::kRaw;
  j.s_ = std::move(json_text);
  return j;
}

Json & Json::set(const std::string & key, Json value)
{
  if (type_ != Type::kObject) {
    *this = object();
  }
  for (auto & kv : members_) {
    if (kv.first == key) {
      kv.second = std::move(value);
      return *this;
    }
  }
  members_.emplace_back(key, std::move(value));
  return *this;
}

Json & Json::push(Json value)
{
  if (type_ != Type::kArray) {
    *this = array();
  }
  items_.push_back(std::move(value));
  return *this;
}

const Json * Json::find(const std::string & key) const
{
  if (type_ != Type::kObject) {
    return nullptr;
  }
  for (const auto & kv : members_) {
    if (kv.first == key) {
      return &kv.second;
    }
  }
  return nullptr;
}

double Json::as_double() const
{
  switch (type_) {
    case Type::kInt:
      return static_cast<double>(i_);
    case Type::kUint:
      return static_cast<double>(u_);
    case Type::kDouble:
      return d_;
    case Type::kBool:
      return b_ ? 1.0 : 0.0;
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

std::string json_quote(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (const char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out.push_back(c);  // UTF-8 bytes pass through unchanged
        }
    }
  }
  out.push_back('"');
  return out;
}

namespace
{
void newline(std::string * out, int indent, int depth)
{
  if (indent <= 0) {
    return;
  }
  out->push_back('\n');
  out->append(static_cast<size_t>(indent * depth), ' ');
}

}  // namespace

std::string json_number(double v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[40];
  // %.10g keeps sub-microsecond resolution for millisecond values without printing
  // 17 noisy digits.
  std::snprintf(buf, sizeof(buf), "%.10g", v);
  return buf;
}

void Json::dump_to(std::string * out, int indent, int depth) const
{
  switch (type_) {
    case Type::kNull:
      out->append("null");
      break;
    case Type::kBool:
      out->append(b_ ? "true" : "false");
      break;
    case Type::kInt:
      out->append(std::to_string(i_));
      break;
    case Type::kUint:
      out->append(std::to_string(u_));
      break;
    case Type::kDouble:
      out->append(json_number(d_));
      break;
    case Type::kString:
      out->append(json_quote(s_));
      break;
    case Type::kRaw:
      out->append(s_.empty() ? "null" : s_);
      break;
    case Type::kArray: {
      if (items_.empty()) {
        out->append("[]");
        break;
      }
      // Arrays of scalars (raw sample rows) stay on one line to keep files small.
      bool scalars = true;
      for (const auto & it : items_) {
        if (it.type_ == Type::kArray || it.type_ == Type::kObject || it.type_ == Type::kRaw) {
          scalars = false;
          break;
        }
      }
      out->push_back('[');
      for (size_t i = 0; i < items_.size(); ++i) {
        if (i) {
          out->push_back(',');
        }
        if (!scalars) {
          newline(out, indent, depth + 1);
        }
        items_[i].dump_to(out, scalars ? 0 : indent, depth + 1);
      }
      if (!scalars) {
        newline(out, indent, depth);
      }
      out->push_back(']');
      break;
    }
    case Type::kObject: {
      if (members_.empty()) {
        out->append("{}");
        break;
      }
      out->push_back('{');
      for (size_t i = 0; i < members_.size(); ++i) {
        if (i) {
          out->push_back(',');
        }
        newline(out, indent, depth + 1);
        out->append(json_quote(members_[i].first));
        out->append(indent > 0 ? ": " : ":");
        members_[i].second.dump_to(out, indent, depth + 1);
      }
      newline(out, indent, depth);
      out->push_back('}');
      break;
    }
  }
}

std::string Json::dump(int indent) const
{
  std::string out;
  dump_to(&out, indent, 0);
  return out;
}

// =============================================================================
// RawTable
// =============================================================================

RawTable::Cell RawTable::Cell::i(int64_t v)
{
  Cell c;
  c.kind = Kind::kInt;
  c.iv = v;
  return c;
}

RawTable::Cell RawTable::Cell::u(uint64_t v)
{
  Cell c;
  c.kind = Kind::kUint;
  c.uv = v;
  return c;
}

RawTable::Cell RawTable::Cell::d(double v)
{
  Cell c;
  c.kind = Kind::kDouble;
  c.dv = v;
  return c;
}

RawTable::Cell RawTable::Cell::null()
{
  return Cell{};
}

RawTable::RawTable(std::vector<std::string> columns, size_t max_rows)
: columns_(std::move(columns)), max_rows_(max_rows)
{
}

void RawTable::add(std::initializer_list<Cell> cells)
{
  if (rows_.size() >= max_rows_) {
    ++dropped_;
    return;
  }
  std::string row = "[";
  bool first = true;
  for (const Cell & c : cells) {
    if (!first) {
      row.push_back(',');
    }
    first = false;
    switch (c.kind) {
      case Cell::Kind::kInt:
        row += std::to_string(c.iv);
        break;
      case Cell::Kind::kUint:
        row += std::to_string(c.uv);
        break;
      case Cell::Kind::kDouble:
        row += json_number(c.dv);
        break;
      case Cell::Kind::kNull:
        row += "null";
        break;
    }
  }
  row.push_back(']');
  rows_.push_back(std::move(row));
}

Json RawTable::to_json() const
{
  Json cols = Json::array();
  for (const auto & c : columns_) {
    cols.push(c);
  }
  Json rows = Json::array();
  for (const auto & r : rows_) {
    rows.push(Json::raw(r));
  }
  return Json::object()
    .set("columns", std::move(cols))
    .set("rows", std::move(rows))
    .set("dropped_rows", dropped_);
}

}  // namespace morai_grpc_vendor::probe
