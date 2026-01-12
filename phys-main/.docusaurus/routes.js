import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '2ee'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'cbb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '8af'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '459'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '548'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '26f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'd43'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', 'a9c'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '96c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/balance-manipulation-and-whole-body-control',
        component: ComponentCreator('/balance-manipulation-and-whole-body-control', '55e'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/capstone-autonomous-humanoid',
        component: ComponentCreator('/capstone-autonomous-humanoid', 'a1b'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/conversational-robotics-voice-to-action',
        component: ComponentCreator('/conversational-robotics-voice-to-action', '86a'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/embodied-intelligence-and-humanoids',
        component: ComponentCreator('/embodied-intelligence-and-humanoids', '5e0'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/introduction-to-physical-ai',
        component: ComponentCreator('/introduction-to-physical-ai', '2e7'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/isaac-ros-and-perception',
        component: ComponentCreator('/isaac-ros-and-perception', '062'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/navigation-and-bipedal-locomotion',
        component: ComponentCreator('/navigation-and-bipedal-locomotion', '09f'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/nvidia-isaac-sim-and-digital-twins',
        component: ComponentCreator('/nvidia-isaac-sim-and-digital-twins', '2dd'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/recommended-humanoid-robots',
        component: ComponentCreator('/recommended-humanoid-robots', '99f'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ros2-the-robotic-nervous-system',
        component: ComponentCreator('/ros2-the-robotic-nervous-system', '0d7'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/sensors-in-physical-ai',
        component: ComponentCreator('/sensors-in-physical-ai', '3eb'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/simulation-with-gazebo',
        component: ComponentCreator('/simulation-with-gazebo', '3ef'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/student-hardware-guide',
        component: ComponentCreator('/student-hardware-guide', '22b'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/urdf-and-robot-description',
        component: ComponentCreator('/urdf-and-robot-description', '5a8'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/vision-language-action-models',
        component: ComponentCreator('/vision-language-action-models', '1d3'),
        exact: true,
        sidebar: "tutorialSidebar"
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
