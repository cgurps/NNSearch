#include "ProgramOptions.h"

#include <iostream>

ProgramOptions parseOptions(int argc, char* argv[])
{
  namespace po = boost::program_options;

  ProgramOptions options;

  po::options_description poLoader("Program options");
  poLoader.add_options()
    ("input,i", po::value<std::string>(&options.filename)->default_value("shapes/bunny.obj"), "input OBJ filepath")
    ("nbTestPoints,n", po::value<std::size_t>(&options.nbTestPoints)->default_value(100), "number of test points")
  ;

  po::options_description po_options("sim [options]");
  po_options.add(poLoader).add_options()
    ("help,h", "display this message")
  ;

  try
  {
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(po_options).run(), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
      std::cout << po_options;
      std::exit(0);
    }
  }
  catch (std::exception& ex)
  {
    std::cout << ex.what() << std::endl;
    std::cout << po_options;
    std::exit(1);
  }

  return options;
}
