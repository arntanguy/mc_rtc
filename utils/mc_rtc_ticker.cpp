#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <chrono>
#include <iostream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  std::string conf;
  bool stepByStep = false;
  bool no_ticker_sync = false;
  double run_for = std::numeric_limits<double>::infinity();
  po::options_description desc("mc-rtc-magnum-ticker options");
  po::positional_options_description p;
  p.add("mc-config", 1);
  // clang-format off
  desc.add_options()
    ("help", "Show this help message")
    ("mc-config", po::value<std::string>(&conf), "Configuration given to mc_rtc")
    ("step-by-step", po::bool_switch(&stepByStep), "Start the ticker in step-by-step mode")
    ("run-for", po::value<double>(&run_for), "Run for the specified time (seconds)")
    ("no-sync", po::bool_switch(&no_ticker_sync), "Disable synchronization with real time");
  // clang-format on
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }
  bool ticker_sync = !no_ticker_sync;
  mc_control::MCGlobalController controller{conf};

  std::vector<double> q;
  {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    for(const auto & jn : rjo)
    {
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
        {
          q.push_back(qj);
        }
      }
      else
      {
        // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
        // likely to be true)
        q.push_back(0);
      }
    }
  }
  controller.setEncoderValues(q);
  controller.init(q);
  controller.running = true;

  size_t nextStep = 0;
  auto toogleStepByStep = [&]() {
    if(stepByStep)
    {
      stepByStep = false;
    }
    else
    {
      nextStep = 0;
      stepByStep = true;
    }
  };
  bool ticker_run = true;
  {
    auto & gui = controller.controller().gui();
    gui.addElement({"Ticker"}, mc_rtc::gui::Button("Stop", [&ticker_run]() { ticker_run = false; }),
                   mc_rtc::gui::Checkbox(
                       "Sync with real-time", [&]() { return ticker_sync; }, [&]() { ticker_sync = !ticker_sync; }),
                   mc_rtc::gui::Checkbox(
                       "Step by step", [&]() { return stepByStep; }, [&]() { toogleStepByStep(); }));
    auto dt = controller.timestep();
    auto buttonText = [&](size_t n) {
      size_t n_ms = static_cast<size_t>(std::ceil(static_cast<double>(n) * 1000.0 * dt));
      return "+" + std::to_string(n_ms) + "ms";
    };
    gui.addElement({"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal,
                   mc_rtc::gui::Button(buttonText(1), [&]() { nextStep = 1; }),
                   mc_rtc::gui::Button(buttonText(5), [&]() { nextStep = 5; }),
                   mc_rtc::gui::Button(buttonText(10), [&]() { nextStep = 10; }),
                   mc_rtc::gui::Button(buttonText(50), [&]() { nextStep = 20; }),
                   mc_rtc::gui::Button(buttonText(100), [&]() { nextStep = 100; }));
  }

  double run_time = 0.0;
  auto runController = [&]() {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    q.resize(rjo.size());
    size_t index = 0;
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & jn = rjo[i];
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
        {
          q[index] = qj;
          index++;
        }
      }
    }
    controller.setEncoderValues(q);
    controller.run();
    if(std::fabs(run_for - run_time) < 1e-6)
    {
      ticker_run = false;
    }
    run_time += controller.controller().solver().dt();
  };
  auto updateGUI = [&]() {
    controller.running = false;
    controller.run();
    controller.running = true;
  };

  using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock,
                                   std::chrono::steady_clock>;
  using duration_us = std::chrono::duration<double, std::micro>;
  auto dt = duration_us(1e6 * controller.timestep());

  // FIXME Catch ctrl^c to interrupt
  while(ticker_run)
  {
    auto now = clock::now();
    if(stepByStep)
    {
      if(nextStep > 0)
      {
        nextStep--;
        runController();
      }
      else
      {
        updateGUI();
      }
    }
    else
    {
      runController();
    }
    if(ticker_sync)
    {
      std::this_thread::sleep_until(now + dt);
    }
  }

  return 0;
}
