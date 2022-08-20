
// Compile with: g++ -Wall -Wextra -fsanitize=address operators.cpp boids.cpp rulesofflight.cpp main.cpp -lsfml-graphics -lsfml-window -lsfml-system
// Execute with: ./a.out
// Build using cmake in debug mode: cmake --build build
// Build using cmake in release mode: cmake --build build_release
// Execute using cmake in debug mode: build/boids-sfml
// Execute using cmake in release mode (suggested): build_release/boids-sfml
// close the window from sfml button parameters: s ~ 1, a ~ 0.5, c ~ 1 for 1 s

#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>

#include "boids.hpp"

int sign(){
  std::srand(time(0));
  int result{};
  if(((std::rand()%10)+1)%2 == 0){
  result = 1;}
  else
  result = -1;
  return result;
}

auto evolve(Boids& boids, int step_evolution, sf::Time delta_t) {
  double const unit_of_t{delta_t.asSeconds()};
  for (int i{0}; i != step_evolution; ++i) {  // attenzione!!!! potrebbe esserci problema di velocità
    boids.evolution(unit_of_t);
 }
  return boids.TotalBoids();
}

auto simulate(Boids& b, double duration, int step_evolution, int prescale){
    std::vector<std::string> b_states;
    double delta_t{duration/step_evolution};
    for (int step = 0; step != step_evolution; ++step){
      if (step % prescale == 0){
        b_states.push_back(state(b, delta_t)); //state of the chain after delta_t
      }
    }
    return b_states;
}


int main() {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<double> pos_x(0, 1179);
  std::uniform_real_distribution<double> pos_y(0, 690);
  std::uniform_real_distribution<double> speed(300, 400);

  std::cout << "Insert number of boids (at least 2): " << '\n'; //fino a 100 tutto ok, poi comincia a laggare
  int n;
  std::cin >> n;
  if (std::cin.fail() || n <= 1) {
    std::cerr << "Invalid number\n";
    return EXIT_FAILURE;
  }
  std::cout
      << "Insert separation const [0,2[, alignment const [0,1[, cohesion const [0, 0.1[: "
      << '\n';  // valori di n ottimali: intorno a 20 per il momento
  double s;
  double a;
  double c;

  std::cin >> s >> a >> c;

  Boids boids{n, 150., SeparationRule{s, 15.}, AlignmentRule{a},
              CohesionRule{c}};

  auto vec_boids = boids.TotalBoids();

  // controllo boids stassa posizione -> capire come fare controllo sulla
  // posizione

  vec_boids.resize(n);

  std::generate(vec_boids.begin(), vec_boids.end(),
                [&gen, &pos_x, &pos_y, &speed]() -> BoidState {
                  return {pos_x(gen), pos_y(gen), speed(gen) * sign(), speed(gen) * sign()};
                });

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



  boids.setvector(vec_boids);

  auto const delta_t{sf::seconds(0.1)};  
  int const fps{30};
  int const step_evolution{1000 / fps}; //quanti step devo fare per star dentro a 30 fps
  // int const prescale{10}; //width of time interval between a measurment and
  // the following

  auto const b_states = simulate(boids, 60.0, 3000, 100);

  std::for_each(b_states.begin(), b_states.end(),
                [](std::string const& state) { std::cout << state << '\n'; });

  std::cout << "State of the boids summary: "
            << state(boids, delta_t.asSeconds()) << '\n';  // at delta_t

  sf::RenderWindow window(sf::VideoMode(1179, 691), "Sea");
  window.setFramerateLimit(fps);
  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    // sollevare un'eccezione eventualmente
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  sf::CircleShape triangle;
  triangle.setRadius(7.f);
  triangle.setPointCount(3);
  triangle.setFillColor(sf::Color(245, 152, 66));

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
    stats.setString(state(boids, delta_t.asSeconds()));

    window.clear();
    window.draw(sprite);
    for (auto& b : boidscopy) {
      triangle.setPosition(b.x, b.y);
      window.draw(triangle);
    }
    window.draw(rect);
    window.draw(stats);

    window.display();
  }
}
