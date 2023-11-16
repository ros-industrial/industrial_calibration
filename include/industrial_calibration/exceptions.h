#pragma once

#include <stdexcept>

namespace industrial_calibration
{
/**
 * @brief Exception for general industrial calibration errors
 */
class ICalException : public std::runtime_error
{
public:
  ICalException(const std::string& what) : std::runtime_error(what) {}
};

/**
 * @brief Exception related specifically to optimization-related errors
 */
struct OptimizationException : public ICalException
{
public:
  using ICalException::ICalException;
};

/**
 * @brief Exception related specifically to covariance-related errors
 */
struct CovarianceException : public ICalException
{
public:
  using ICalException::ICalException;
};

/**
 * \brief Thrown during errors in loading or parsing data files
 */
class BadFileException : public ICalException
{
public:
  using ICalException::ICalException;
};

}  // namespace industrial_calibration
