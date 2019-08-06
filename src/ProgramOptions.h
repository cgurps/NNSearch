#pragma once

/**
 * @file ProgramOptions.h
 * @author Thomas Caissard (\c thomas.caissard@gmail.com)
 * @date 2019/08/05
 */

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/errors.hpp>

#include <Eigen/Dense>

/**
 * @struct ProgramOptions
 * @brief The interface for the program option
 */
struct ProgramOptions
{
  /**
   * Filename for the OBJ loader
   */
  std::string filename;

  /**
   * Number of test points for the main scope of the application
   */
  std::size_t nbTestPoints;

  /**
   * Point for the query executable
   */
  Eigen::Matrix<double,3,1> point;
};

/**
 * Parse the input option and return the initialized ProgramOptions
 */
ProgramOptions parseOptions(int argc, char* argv[]);
