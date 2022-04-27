//
// Created by SENSETIME\yezhichao1 on 2020/4/22.
//
#pragma once
#ifndef WARG_TIMER_H
#define WARG_TIMER_H

#endif  // WARG_TIMER_H

#include <chrono>
#include <string>
#include <utility>

#define TIMING(a, b) \
  {                  \
    a.resume();      \
    b;               \
    a.stop();        \
  };

class Timer {
 private:
  using _Clock = std::chrono::high_resolution_clock;
  std::string format;
  double time_count;
  std::chrono::time_point<_Clock> start_time;

 public:
  Timer() : time_count(0){};

  Timer(std::string _format) : format(std::move(_format)), time_count(0){};

  void init(std::string _format) { format = std::move(_format); }

  void start() {
    time_count = 0;
    start_time = _Clock::now();
  };

  void stop() {
    auto stop_time = _Clock::now();
    time_count += std::chrono::duration_cast<std::chrono::duration<double>>(stop_time - start_time).count();
  }

  void resume() { start_time = _Clock::now(); }

  void print() { printf(format.c_str(), time_count); };

  void log(std::ofstream &time_log_stream) {
    char buffer[128];
    sprintf(buffer, format.c_str(), time_count);
    time_log_stream << buffer;
  }
};

struct TimerArray {
  Timer tot, reg, tri, fil, che, lba, gba, merge;
  std::vector<Timer *> timer_vec;
  TimerArray() {
    tot = Timer("Time Total %.6lfs\n");
    reg = Timer("Time reg %.6lfs\n");
    tri = Timer("Time tri %.6lfs\n");
    fil = Timer("Time fil %.6lfs\n");
    merge = Timer("Time merge %.6lfs\n");
    che = Timer("Time check %.6lfs\n");
    lba = Timer("Time LBA %.6lfs\n");
    gba = Timer("Time GBA %.6lfs\n");
    timer_vec = {&tot, &reg, &tri, &fil, &merge, &che, &lba, &gba};
  }
};
