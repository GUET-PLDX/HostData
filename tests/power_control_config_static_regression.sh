#!/usr/bin/env bash

set -euo pipefail

ruby <<'RUBY'
require "yaml"

MANIFEST_PATH = "Modules/PowerControl/PowerControl.hpp"
EXPECTED = {
  "User/RobotConfig/sentry_chassis.yaml" => ["@&super_power", 5.5, 4, 0],
}.freeze
NAMES = %w[
  superpower
  chassis_static_power_loss
  motor_count_3508
  motor_count_6020
].freeze

failures = []
check = ->(condition, message) { failures << message unless condition }

source = File.open(MANIFEST_PATH, "r:bom|utf-8", &:read)
match = source.match(
  %r{/\* === MODULE MANIFEST V2 ===(?<yaml>.*?)=== END MANIFEST === \*/}m,
)
check.call(!match.nil?, "#{MANIFEST_PATH}: missing module manifest")
abort failures.join("\n") if match.nil?

manifest = YAML.safe_load(match[:yaml], aliases: false)
manifest_args = manifest.fetch("constructor_args").map { |entry| entry.keys.first }
check.call(
  manifest_args == NAMES,
  "#{MANIFEST_PATH}: expected #{NAMES.inspect}, got #{manifest_args.inspect}",
)

found = {}
Dir["User/RobotConfig/*.yaml"].sort.each do |path|
  document = YAML.safe_load(File.open(path, "r:bom|utf-8", &:read), aliases: true)
  modules = document.is_a?(Hash) ? document.fetch("modules", []) : []
  instances = modules.select { |entry| entry.is_a?(Hash) && entry["name"] == "PowerControl" }
  found[path] = instances unless instances.empty?
end

check.call(found.keys.sort == EXPECTED.keys.sort,
           "PowerControl instances differ: #{found.keys.sort.inspect}")
EXPECTED.each do |path, values|
  instances = found.fetch(path, [])
  check.call(instances.length == 1,
             "#{path}: expected one PowerControl, got #{instances.length}")
  next unless instances.length == 1

  args = instances.first.fetch("constructor_args", {})
  check.call(args.keys == NAMES,
             "#{path}: expected only #{NAMES.inspect}, got #{args.keys.inspect}")
  expected = NAMES.zip(values).to_h
  check.call(args == expected, "#{path}: expected #{expected.inspect}, got #{args.inspect}")
end

module_readme = File.open(
  "Modules/PowerControl/README.md", "r:bom|utf-8", &:read
).delete("`").gsub(/\s+/, " ")
root_readme = File.open("README.md", "r:bom|utf-8", &:read).delete("`").gsub(/\s+/, " ")
check.call(module_readme.match?(/RLS\.hpp.*PowerControl\.hpp/i),
           "module README must document the two-header structure")
check.call(module_readme.match?(/Normal.*Boost/i),
           "module README must document Normal/Boost")
check.call(module_readme.match?(/LibXR::PID.*P=50.*D=0\.2/i),
           "module README must document the LibXR PD energy controllers")
check.call(module_readme.match?(/immediate recovery/i),
           "module README must document immediate recovery")
check.call(module_readme.match?(/0x0201.*last valid/i),
           "module README must document the last-valid 0x0201 policy")
check.call(root_readme.match?(/PowerControl centrally owns one power budget/i),
           "root README must document centralized budget ownership")
check.call(root_readme.match?(/centralized freshness/i),
           "root README must document centralized freshness ownership")
check.call(root_readme.match?(/immediate recovery/i),
           "root README must document immediate recovery")

unless failures.empty?
  warn failures.map { |failure| "FAIL: #{failure}" }.join("\n")
  exit 1
end

puts "PASS: PowerControl minimal configuration contract"
RUBY
