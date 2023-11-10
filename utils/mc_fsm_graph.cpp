/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/TransitionMap.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/config.h>
#include <boost/program_options.hpp>
#include <fstream>
namespace po = boost::program_options;

void usage()
{
  std::cout << "mc_fsm_graph is a command-line tool to generate graphviz dot files from FSM transition maps\n\n";
  std::cout << "\nUse mc_fsm_graph <config_in> <dot_out>\n";
}

int main(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_fsm_graph show options");
  auto config_path = std::vector<std::string>{};
  auto state = std::string{};
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input file")
    ("out", po::value<std::string>(), "Output file")
    ("config_path", po::value<std::vector<std::string>>(&config_path)->multitoken(), "Path to the transitions map")
    ("state", po::value<std::string>(&state), "State to display");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);

  if((!vm.count("in") && !vm.count("out")) || vm.count("help"))
  {
    usage();
    return !vm.count("help");
  }
  auto in = vm["in"].as<std::string>();
  auto out = vm["out"].as<std::string>();

  if(config_path.size() && state.size())
  {
    mc_rtc::log::error("state and config_path options cannot be used simultaneously");
    exit(-1);
  }

  auto conf = mc_rtc::Configuration(in);
  auto factory = mc_control::fsm::StateFactory(conf("StatesLibraries"), conf("StatesFiles"), false);

  auto states = conf("states", mc_rtc::Configuration{});
  factory.load(states);

  mc_control::fsm::TransitionMap transitions;
  mc_rtc::Configuration c = conf;
  if(config_path.size())
  {
    for(const auto & key : config_path) { c = c(key); }
  }
  else if(state.size())
  {
    if(factory.hasState(state)) { c = factory.configuration(state).config; }
    else
    {
      mc_rtc::log::error("No state named {} in this FSM", state);
      exit(-1);
    }
  }
  transitions.init(factory, c);
  std::ofstream file(out);
  if(file) { transitions.printGraphViz(file); }
  else { mc_rtc::log::error("Cannot write to output file {}", out); }
  file.close();
  mc_rtc::log::success("Generated graph in {}", out);
  return 0;
}
