#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include "boids.hpp"

auto evolve(Boids& boids, int step_evolution,
            sf::Time delta_t) {  // delta_t = 0.001 s
  double const unit_of_t{delta_t.asMilliseconds() /
                         10.};  // conversion of time from seconds to double
                                // (0.001 s == 0.1 double)
  for (int i{0}; i != step_evolution; ++i) {
    boids.evolution(unit_of_t);
  }
  return boids.totalboids();
}

std::vector<std::string> simulate(Boids& b, double duration, int steps,
                                  int prescale) {
  std::vector<std::string> b_states;
  double delta_t{(duration / steps) * 100};  // conversion of time
  for (int step = 0; step != steps; ++step) {
    if (step % prescale == 0) {  // sampling
      b_states.push_back(state(b, delta_t));
    } else {
      b.evolution(delta_t);
    }
  }
  return b_states;
}

int main() {
  std::random_device rd;  // set seed
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<double> pos_x(0, 1179);
  std::uniform_real_distribution<double> pos_y(0, 690);
  std::uniform_real_distribution<double> speed(-10, 10);

  std::cout << "Insert number of boids (at least 2): " << '\n';
  int n;
  std::cin >> n;
  if (std::cin.fail() || n <= 1) {
    std::cerr << "Invalid number\n";
    return EXIT_FAILURE;
  }
  std::cout << "Insert separation const [0,2[, alignment const [0,1[, cohesion "
               "const [0, 0.1[: "
            << '\n';
  double s;
  double a;
  double c;

  std::cin >> s >> a >> c;

  Boids boids{n, 150., SeparationRule{s, 15.}, AlignmentRule{a},
              CohesionRule{c}};

  auto vec_boids = boids.totalboids();

  vec_boids.resize(n);
  // fill vector of BoidState using random generation
  std::generate(vec_boids.begin(), vec_boids.end(),
                [&gen, &pos_x, &pos_y, &speed]() -> BoidState {
                  return {pos_x(gen), pos_y(gen), speed(gen), speed(gen)};
                });

  // check if there are boids with the same position and replace them
  auto it = vec_boids.begin();
  for (; it != vec_boids.end(); ++it) {
    auto c = *it;
    auto next = std::next(it);
    auto same_pos_it = std::find_if(
        next, vec_boids.end(),
        [&c](BoidState const& n) { return c.x == n.x && c.y == n.y; });
    if (same_pos_it != vec_boids.end()) {
      vec_boids.erase(same_pos_it);
      vec_boids.push_back(
          BoidState{pos_x(gen), pos_y(gen), speed(gen), speed(gen)});
    }
  }

  assert(same_pos_check(vec_boids));
  boids.setvector(vec_boids);

  auto const delta_t{sf::seconds(0.001)};
  int const fps{30};
  int const step_evolution{1000 / fps};  // how many steps I have to do to
                                         // "synchronize" evolution with fps

  // simulation of 120 s, sampling every two seconds
  auto const b_states = simulate(boids, 120., 120000, 2000);

  std::for_each(b_states.begin(), b_states.end(),
                [](std::string const& state) { std::cout << state << '\n'; });

  sf::RenderWindow window(sf::VideoMode(1179, 691), "Sea");
  window.setFramerateLimit(fps);
  // cosmetic
  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    std::cout << "Failed to load texture" << '\n';
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  // creating "fish"
  sf::CircleShape triangle;
  triangle.setRadius(7.f);
  triangle.setPointCount(3);
  triangle.setFillColor(sf::Color(245, 152, 66));

  // box statistic and cosmetic
  sf::Font font;
  font.loadFromFile("cmuntt.ttf");
  sf::Text stats;
  stats.setFont(font);
  stats.setCharacterSize(15);
  stats.setPosition(sf::Vector2f(680., 0.f));
  sf::RectangleShape rect;
  rect.setPosition(sf::Vector2f(675, 0.));
  rect.setSize(sf::Vector2f(600., 40));
  rect.setFillColor(sf::Color::Black);

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    auto boidscopy = evolve(boids, step_evolution, delta_t);
    stats.setString(state(boids, delta_t.asSeconds() * 100));

    window.clear();
    window.draw(sprite);
    std::for_each(boidscopy.begin(), boidscopy.end(), [&](BoidState const& b) {
      triangle.setPosition(b.x, b.y);
      window.draw(triangle);
    });

    window.draw(rect);
    window.draw(stats);

    window.display();
  }
}
