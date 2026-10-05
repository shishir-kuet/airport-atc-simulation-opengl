// Keyboard and mouse: which key does what, and the camera controls that act
// while a key is held down.
#pragma once

struct GLFWwindow;

// Register the key, mouse, scroll and resize callbacks on the window.
void installCallbacks(GLFWwindow* window);

// Camera keys that repeat every frame (arrows: look around, W/S: zoom).
void processHeldKeys(GLFWwindow* window, float dt);

void selectView(int index);                 // views 0-6
void updateTitle(GLFWwindow* window);       // view name + simulation status
void printControls();                       // the key list printed at start-up
