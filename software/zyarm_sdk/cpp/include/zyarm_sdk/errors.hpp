#pragma once

#include <stdexcept>
#include <string>

namespace zyarm_sdk
{

class ZyArmError : public std::runtime_error
{
public:
  explicit ZyArmError(const std::string & message) : std::runtime_error(message) {}
};

class ProtocolError : public ZyArmError
{
public:
  explicit ProtocolError(const std::string & message) : ZyArmError(message) {}
};

class TransportError : public ZyArmError
{
public:
  explicit TransportError(const std::string & message) : ZyArmError(message) {}
};

class TimeoutError : public ZyArmError
{
public:
  explicit TimeoutError(const std::string & message) : ZyArmError(message) {}
};

class StaleStateError : public ZyArmError
{
public:
  explicit StaleStateError(const std::string & message) : ZyArmError(message) {}
};

class SafetyError : public ZyArmError
{
public:
  explicit SafetyError(const std::string & message) : ZyArmError(message) {}
};

}  // namespace zyarm_sdk
