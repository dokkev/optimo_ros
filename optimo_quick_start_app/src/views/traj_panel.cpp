// Copyright 2024 Roboligent, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "optimo_quick_start_app/views/traj_panel.hpp"

#include <QCoreApplication>
#include <QProgressDialog>

namespace optimo_quick_start_app
{

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// TrajPanel /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

TrajPanel::TrajPanel(QWidget * parent)
: QWidget(parent),
  disable_btn(new QPushButton(this)),
  stop_btn(new QPushButton(this)),
  move_home_btn(new QPushButton(this)),
  teach_btn(new QPushButton(this)),
  play_traj_btn(new QPushButton(this)),
  free_btn(new QPushButton(this))
{
  disable_btn->setText("Disable");
  stop_btn->setText("Stop Motion");
  move_home_btn->setText("Move Home");
  teach_btn->setText("Teach");
  play_traj_btn->setText("Play");
  free_btn->setText("Free Motion");

  connect(disable_btn, &QPushButton::clicked, this, &TrajPanel::disable);
  connect(stop_btn, &QPushButton::clicked, this, &TrajPanel::stop_motion);
  connect(move_home_btn, &QPushButton::clicked, this, &TrajPanel::move_home);
  connect(teach_btn, &QPushButton::clicked, this, &TrajPanel::teach);
  connect(play_traj_btn, &QPushButton::clicked, this, &TrajPanel::play_traj);
  connect(free_btn, &QPushButton::clicked, this, &TrajPanel::free_motion);

  auto btn_layout = new QHBoxLayout;
  btn_layout->addStretch(1);
  btn_layout->addWidget(disable_btn);
  btn_layout->addWidget(stop_btn);
  btn_layout->addWidget(move_home_btn);
  btn_layout->addWidget(teach_btn);
  btn_layout->addWidget(play_traj_btn);
  btn_layout->addWidget(free_btn);
  btn_layout->addStretch(1);

  auto layout = new QVBoxLayout;

  layout->addStretch(1);
  layout->addLayout(btn_layout);
  layout->addStretch(1);

  setLayout(layout);
}

}  // namespace optimo_quick_start_app
