// Compile with: g++ -Wall -Wextra -fsanitize=address main.cpp -lsfml-graphics
// -lsfml-window -lsfml-system close the window from sfml button
#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <iostream>
#include <random>
#include <thread>


auto evolve(Boids& boids, int spevolution,  sf::Time delta_t){
  double const unitoft{delta_t.asSeconds()};
  for(int i{0}; i!=spevolution; ++i){
    boids.evolution(unitoft);
  }
  return boids.TotalBoids();
}

/* auto prova (Boids& v, int stepevolution){
  for(int i{0}; i!=stepevolution; ++i){
   for(int i{0}; i != size(v.TotalBoids()); ++i){
    v.TotalBoids()[i] += {0.1,0.1,0.0,0.0};
   }

  }
  return v.TotalBoids();
} */

int main() {
  std::random_device red;
  std::default_random_engine gen(red());
  std::uniform_real_distribution<double> dist(0, 600);
  std::uniform_real_distribution<double> dist2(0, 100);

  std::cout << "insert number of boids" << '\n';
  int n;
  std::cin >> n;
  std::cout << "Insert s,a,c" << '\n';
  double a;
  double b;
  double c;

  std::cin >> a >> b >> c;

  Boids bob{n, 15., SeparationRule{a, 15. / 10.}, AllignmentRule{b},
            CohesionRule{c}};
  auto bob1 = bob.TotalBoids();
  bob1.resize(n);

  std::generate(bob1.begin(), bob1.end(), [&gen, &dist, &dist2]() -> BoidState {
    return {dist(gen), dist(gen), dist2(gen), dist2(gen)};
  });
  //std::cout << bob1[1].x <<'\n'; 

  bob.setvector(bob1); 

  //std::cout << (bob.TotalBoids())[1].x << '\n';

  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const step_evolution{300 / fps};

  sf::RenderWindow window(sf::VideoMode(1179, 691), "sperem");
  window.setFramerateLimit(fps);
  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    // sollevare un'eccezione eventualmente
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  sf::CircleShape rec;
  rec.setRadius(10.f);
  rec.setPointCount(3);

  rec.setFillColor(sf::Color(245, 152, 66));

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    window.clear();
    window.draw(sprite);
    //auto bobcopy = bob1; //va
    // auto bobcopy = prova(bob, step_evolution);
    auto bobcopy = evolve(bob, step_evolution, delta_t);

    for (auto& bobbysss : bobcopy) {
      rec.setPosition(bobbysss.x, bobbysss.y);
      window.draw(rec);
    }


    window.display();
  }
}
