// Compile with: g++ -Wall -Wextra -fsanitize=address main.cpp -lsfml-graphics -lsfml-window -lsfml-system 

//close the window from sfml button
#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <iostream>
#include <random>
#include <thread>


auto evolve(Boids& boids, int spevolution, sf::Time delta_t) {
  double const unitoft{delta_t.asSeconds()};
  for (int i{0}; i != spevolution; ++i) {
    boids.evolution(unitoft);
  }
  return boids.TotalBoids();
}

int main() {
  std::random_device red;
  std::default_random_engine gen(red());
  std::uniform_real_distribution<double> dist(0, 690);
  std::uniform_real_distribution<double> dist2(0, 50);

  std::cout << "insert number of boids" << '\n';
  int n;
  std::cin >> n;
  std::cout
      << "Insert separation const, allignement const ( < 1 ), cohesion const "
      << '\n';  // valori di n ottimali: intorno a 20 per il momento
  double a;
  double b;
  double c;

  std::cin >> a >> b >> c;

  Boids bob{n, 20., SeparationRule{a, 20. / 10.}, AllignmentRule{b},
            CohesionRule{c}};
  auto bob1 = bob.TotalBoids();
  bob1.resize(n);

  std::generate(bob1.begin(), bob1.end(), [&gen, &dist, &dist2]() -> BoidState {
    return {dist(gen), dist(gen), dist2(gen), dist2(gen)};
  });
  // std::cout << bob1[1].x <<'\n';

  bob.setvector(bob1);

  state(bob, 0.1);

  // std::cout << (bob.TotalBoids())[1].x << '\n';

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

   /*  for (int i{0}; i != size(bob.TotalBoids()); ++i) {
      if ((bob.TotalBoids())[i].x < 0. || (bob.TotalBoids())[i].x > 691. ||
          (bob.TotalBoids())[i].y < 0. || (bob.TotalBoids())[i].y > 1179.) {
        if ((bob.TotalBoids())[i].x < 0. || (bob.TotalBoids())[i].x > 691.) {
          (bob.TotalBoids())[i].v_x = -(bob.TotalBoids())[i].v_x;
        } else {
          (bob.TotalBoids())[i].v_y = -(bob.TotalBoids())[i].v_y;
        }
      }
    } */
    // sputato a un video su youtube
    /* if (rec.getPosition().x < 0.f ){
      rec.setPosition(0.f, rec.getPosition().y);
    }
    if (rec.getPosition().x + rec.getGlobalBounds().width > 1179 ){
      rec.setPosition(1179.f- rec.getGlobalBounds().width, rec.getPosition().y);
    }
    if (rec.getPosition().y < 0.f ){
      rec.setPosition(rec.getPosition().x, 0.f);
    }
    if (rec.getPosition().y + rec.getGlobalBounds().height >691 ){
      rec.setPosition(rec.getPosition().x, 691.f- rec.getGlobalBounds().height);
    } */

    window.clear();
    window.draw(sprite);
    // auto bobcopy = bob1; //va
    auto bobcopy = evolve(bob, step_evolution, delta_t);

    for (auto& bobbysss : bobcopy) {
      rec.setPosition(bobbysss.x, bobbysss.y);
      window.draw(rec);
    }

    window.display();
  }
}
