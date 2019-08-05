#pragma once

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/errors.hpp>

struct ProgramOptions
{
  std::string filename;
  std::size_t nbTestPoints;
};

ProgramOptions parseOptions(int argc, char* argv[]);
