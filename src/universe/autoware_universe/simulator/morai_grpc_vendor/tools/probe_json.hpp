// 신규 — 26.09.24 _ YSH _ morai_probe 결과용 최소 JSON 값 트리 (외부 의존 없음)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_JSON_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_JSON_HPP_

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::probe
{

/// Insertion-ordered JSON value. Only what the probe needs: no parsing.
///
/// int64 is kept distinct from double because MORAI timestamps may be epoch ns
/// (~1.7e18), which a double cannot represent exactly. Non-finite doubles serialise as
/// null. `raw()` embeds text that is already JSON (protobuf MessageToJsonString output).
class Json
{
public:
  enum class Type { kNull, kBool, kInt, kUint, kDouble, kString, kArray, kObject, kRaw };

  Json() = default;
  // Fixed-width overloads cover int, int64 (protobuf int64 == int64_t), unsigned and
  // size_t on LP64 without ambiguity.
  Json(bool v);          // NOLINT(runtime/explicit)
  Json(int32_t v);       // NOLINT(runtime/explicit)
  Json(int64_t v);       // NOLINT(runtime/explicit)
  Json(uint32_t v);      // NOLINT(runtime/explicit)
  Json(uint64_t v);      // NOLINT(runtime/explicit)
  Json(double v);        // NOLINT(runtime/explicit)
  Json(const char * v);  // NOLINT(runtime/explicit)
  Json(std::string v);   // NOLINT(runtime/explicit)

  static Json object();
  static Json array();
  static Json raw(std::string json_text);

  /// Object: append (or overwrite) `key`. Returns *this for chaining.
  Json & set(const std::string & key, Json value);
  /// Array: append.
  Json & push(Json value);

  Type type() const { return type_; }
  /// Object lookup; nullptr if absent or not an object.
  const Json * find(const std::string & key) const;
  double as_double() const;

  std::string dump(int indent = 2) const;

private:
  void dump_to(std::string * out, int indent, int depth) const;

  Type type_{Type::kNull};
  bool b_{false};
  int64_t i_{0};
  uint64_t u_{0};
  double d_{0.0};
  std::string s_;
  std::vector<std::pair<std::string, Json>> members_;  // object
  std::vector<Json> items_;                            // array
};

/// JSON string-literal escaping (quotes included).
std::string json_quote(const std::string & s);

/// JSON number text for a double: "%.10g", non-finite -> "null".
std::string json_number(double v);

/// Raw per-sample rows, pre-formatted as text so that a long tight loop (up to hundreds
/// of thousands of rows) does not cost one Json node per cell. Rows beyond `max_rows` are
/// counted, not stored.
class RawTable
{
public:
  struct Cell
  {
    enum class Kind { kInt, kUint, kDouble, kNull };
    static Cell i(int64_t v);
    static Cell u(uint64_t v);
    static Cell d(double v);
    static Cell null();
    Kind kind{Kind::kNull};
    int64_t iv{0};
    uint64_t uv{0};
    double dv{0.0};
  };

  RawTable(std::vector<std::string> columns, size_t max_rows);

  void add(std::initializer_list<Cell> cells);
  size_t rows() const { return rows_.size(); }
  size_t dropped() const { return dropped_; }

  /// {"columns": [...], "rows": [[...], ...], "dropped_rows": n}
  Json to_json() const;

private:
  std::vector<std::string> columns_;
  size_t max_rows_;
  size_t dropped_{0};
  std::vector<std::string> rows_;
};

}  // namespace morai_grpc_vendor::probe

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_JSON_HPP_
