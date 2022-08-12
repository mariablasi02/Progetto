// Compile with: g++ main.cpp -lsfml-graphics -lsfml-window -lsfml-system

#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <iostream>
#include <random>

#include "boids.hpp"

auto evolution(Boids& pesci, int steps_per_evolution, sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    pesci.evolution(dt);
  }

  return pesci.state();
}

int main() {
  std::cout << "Enter number of boids: " << '\n';
  int n;
  std::cin >> n;
  std::cout << "Enter parameters of separation, allignment (between 0 and 1), "
               "cohesion: "
            << '\n';
  double s;
  double a;
  double c;
  std::cin >> s >> a >> c;
  double d = 15.;
  Boids boids{n, d, SeparationRule{s, d / 10}, AllignmentRule{a},
              CohesionRule{c}};

  std::random_device rd;  // seed
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<> pos(0, 500);
  std::uniform_real_distribution<> speed(0, 10);

  (boids.TotalBoids()).resize(n);
  std::generate((boids.TotalBoids()).begin(), (boids.TotalBoids()).end(),
                [&pos, &speed, &gen]() -> BoidState {
                  return {pos(gen), pos(gen), speed(gen), speed(gen)};
                });

  

  // something
  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const steps_per_evolution{1000 / fps};

  unsigned const display_width = .9 * sf::VideoMode::getDesktopMode().width;
  unsigned const display_height = .9 * sf::VideoMode::getDesktopMode().height;
  sf::RenderWindow window(sf::VideoMode(display_width, display_height), "Mare");
  window.setFramerateLimit(fps);

  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    // sollevare un'eccezione eventualmente
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  sf::CircleShape fish;
  fish.setRadius(8.0f);
  fish.setPointCount(3);
  fish.setFillColor(sf::Color(245, 152, 66));

  std::vector<sf::CircleShape> fishes;
  fishes.resize(n);
  auto it = (boids.TotalBoids()).begin();
  int i = 0;
  for (; i != n && it != (boids.TotalBoids()).end(); ++i, ++it) {
    fishes.push_back(fish);
    fish.setPosition(static_cast<float>((*it).x), static_cast<float>((*it).y));
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {
        window.close();
      }
    }
    window.clear();
    window.draw(sprite);
    for (auto& fish : fishes) {
      window.draw(fish);
    }
    // draw everything here
    // fish.setPosition(500, 400);
    // window.draw(fish);
    window.display();  // takes what was drawn since the last call to display
                       // and displays it on the window.
  }
}