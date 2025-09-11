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

#include "optimo_quick_start_app/views/jog_panel.hpp"

#include <QCoreApplication>
#include <QProgressDialog>

namespace optimo_quick_start_app
{
JogPanel::JogPanel(QWidget * parent)
: QWidget(parent),
  stop_btn(new QPushButton(this)),
  jog_btn(new QPushButton(this)),
  grasp_btn(new QPushButton(this)),
  relax_btn(new QPushButton(this)),
  release_btn(new QPushButton(this))
{
  stop_btn->setText("Stop Motion");
  jog_btn->setText("Jog");
  grasp_btn->setText("Grasp");
  relax_btn->setText("Relax");
  release_btn->setText("Release");

  connect(stop_btn, &QPushButton::clicked, this, &JogPanel::stop_motion);

  auto btn_layout = new QHBoxLayout;
  btn_layout->addStretch(1);
  btn_layout->addWidget(stop_btn);
  btn_layout->addWidget(jog_btn);
  btn_layout->addWidget(grasp_btn);
  btn_layout->addWidget(relax_btn);
  btn_layout->addWidget(release_btn);
  btn_layout->addStretch(1);

  auto layout = new QVBoxLayout;

  layout->addStretch(1);
  layout->addLayout(btn_layout);
  layout->addStretch(1);

  setLayout(layout);
}

}  // namespace optimo_quick_start_app
