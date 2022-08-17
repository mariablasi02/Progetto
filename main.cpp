// Compile with: g++ -Wall -Wextra -fsanitize=address operators.cpp boids.cpp rulesofflight.cpp main.cpp -lsfml-graphics -lsfml-window -lsfml-system 

//close the window from sfml button

// parameters: s= 0.001, a = 0.9, c= 0.03  for 1 ms
#include "boids.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <iostream>
#include <random>
#include <cstdlib>


auto evolve(Boids& boids, int sp_evolution, sf::Time delta_t) {
  double const unit_of_t{delta_t.asSeconds()};
  for (int i{0}; i != sp_evolution; ++i) { // attenzione!!!! potrebbe esserci problema di velocità
    boids.evolution(unit_of_t);
  }
  return boids.TotalBoids();
}

int main() {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_real_distribution<double> pos_x(0, 1179);
  std::uniform_real_distribution<double> pos_y(0, 690);
  std::uniform_real_distribution<double> speed(-100, 100);

  std::cout << "Insert number of boids (at least 2): " << '\n';
  int n;
  std::cin >> n;
  if(std::cin.fail()|| n<=1){ 
    std::cerr << "Invalid number\n";
    return EXIT_FAILURE;
  }
  std::cout
      << "Insert separation const, alignment const ( < 1 ), cohesion const: "
      << '\n';  // valori di n ottimali: intorno a 20 per il momento
  double s;
  double a;
  double c;

  std::cin >> s >> a >> c;
  
  Boids boids{n, 300., SeparationRule{s, 25.}, AlignmentRule{a},
            CohesionRule{c}};

  auto vec_boids = boids.TotalBoids();

//controllo boids stassa posizione -> capire come fare controllo sulla posizione

  vec_boids.resize(n);
  
  std::generate(vec_boids.begin(), vec_boids.end(), [&gen, &pos_x, &pos_y, &speed]() -> BoidState {
    return {pos_x(gen), pos_y(gen), speed(gen), speed(gen)};
  });


  // std::cout << bob1[1].x <<'\n';

  boids.setvector(vec_boids);


  // std::cout << (bob.TotalBoids())[1].x << '\n';


  auto const delta_t{sf::milliseconds(500)};

  int const fps = 30;
  int const step_evolution{3000 / fps};

  std::cout << state(boids, delta_t.asSeconds()) <<'\n';
 

  sf::RenderWindow window(sf::VideoMode(1179, 691), "Sea");
  window.setFramerateLimit(fps);
  sf::Texture texture;
  if (!texture.loadFromFile("sfondomare.png")) {
    // sollevare un'eccezione eventualmente
  }
  sf::Sprite sprite;
  sprite.setTexture(texture);

  sf::CircleShape triangle;
  triangle.setRadius(10.f);
  triangle.setPointCount(3);
  triangle.setFillColor(sf::Color(245, 152, 66));

  
  sf::Font font;
  font.loadFromFile("cmuntt.ttf");
  sf::Text stats;
  stats.setFont(font);
  //stats.setString(state(boids, delta_t.asSeconds()));
  stats.setCharacterSize(15);
  stats.setPosition(sf::Vector2f(680.,0.f));
  sf::RectangleShape rect;
  rect.setPosition(sf::Vector2f(675, 0.));
  rect.setSize(sf::Vector2f(600.,40));
  rect.setFillColor(sf::Color::Black);
  

  while (window.isOpen())  {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    auto boidscopy = evolve(boids, step_evolution, delta_t);
    stats.setString(state(boids, delta_t.asSeconds()));
    //int i = 0;

    window.clear();
    window.draw(sprite);
    for(auto& b : boidscopy){
      triangle.setPosition(b.x, b.y);
      
      window.draw(triangle);
  
    }
    window.draw(rect);
    window.draw(stats);
    


    window.display();
  }

}
