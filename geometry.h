#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

struct Point {
  Point() = default;
  Point(double x, double y) : x(x), y(y) {}
  ~Point() = default;
  bool operator==(const Point& other) const {
	const double kEps = 0.00001;
	return (fabs(this->x - other.x) < kEps && fabs(this->y - other.y) < kEps);
  };
  bool operator!=(const Point& other) const { return !(*this == other); }

  double x = 0.f;
  double y = 0.f;
};

double Distance(const Point& p1, const Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

class Line {
 public:
  Line() = default;
  Line(const Point& first_point, const Point& second_point) {
	if (first_point.x == second_point.x) {
	  angle = std::numeric_limits<double>::max();
	} else {
	  angle = (first_point.y - second_point.y) / (first_point.x - second_point.x);
	}
	shift = first_point.y - angle * first_point. x;
  };
  Line(double angle, double shift) : angle(angle), shift(shift) {}
  Line(const Point& point, double angle) : angle(angle), shift(point.y - angle * point.x) {}
  ~Line() = default;
  Line& operator=(const Line&) = default;
  bool operator==(const Line& other) const {
	return (fabs(this->angle - other.angle) < kEps && fabs(this->shift - other.shift) < kEps);
  }
  bool operator!=(const Line& other) const { return !(*this == other); }

  constexpr static const double kEps = 0.0000001;
  double angle = 0.f;
  double shift = 0.f;
};

double tan(const Line& l1, const Line& l2) {
  return fabs(std::min(fabs(l1.angle - l2.angle) / (1 + l1.angle * l2.angle), std::numeric_limits<double>::max()));
}

class Shape {
 public:
  virtual double perimeter() const = 0;
  virtual double area() const = 0;
  virtual bool containsPoint(const Point& point) const = 0;
  virtual bool operator==(const Shape& other) const = 0;
  virtual bool operator!=(const Shape& other) const = 0;
  virtual bool isCongruentTo(const Shape& other) const = 0;
  virtual bool isSimilarTo(const Shape& other) const = 0;
  virtual void rotate(const Point& center, double angle) = 0;
  virtual void reflect(const Point& center) = 0;
  virtual void reflect(const Line& axis) = 0;
  virtual void scale(const Point& center, double coefficient) = 0;
  virtual ~Shape() = default;
};

class Polygon : public Shape {
 public:
  Polygon() = default;
  Polygon(const std::vector<Point>& points) : vertices_(points) {}
  template <typename... Args>
  Polygon(Args... args) : vertices_({args...}) {}
  size_t verticesCount() const { return vertices_.size(); }
  const std::vector<Point>& getVertices() const { return vertices_; }
  bool isConvex() const;
  double perimeter() const final;
  double area() const final;
  bool containsPoint(const Point& point) const final;
  bool operator==(const Shape& other) const final;
  bool operator!=(const Shape& other) const final { return !(*this == other); };
  bool isCongruentTo(const Shape& other) const final;
  bool isSimilarTo(const Shape& other) const final;
  void rotate(const Point& center, double angle) final;
  void reflect(const Point& center) final;
  void reflect(const Line& axis) final;
  void scale(const Point& center, double coefficient) final;

 protected:
  std::vector<Point> vertices_;
};

bool Polygon::isConvex() const {
  if (vertices_.size() <= 3) {
	return true;
  }
  bool side;
  Line ex(vertices_[0], vertices_[1]);
  if (vertices_[0].x == vertices_[1].x) {
	side = (vertices_[1].y > vertices_[0].y == vertices_[2].x > vertices_[0].x);
  } else {
	side = (vertices_[0].x < vertices_[1].x == vertices_[2].y < ex.angle * vertices_[2].x + ex.shift);
  }
  for (size_t i = 0; i < vertices_.size(); ++i) {
	Point p1 = vertices_[i];
	Point p2 = vertices_[(i + 1) % vertices_.size()];
	Line line(p1, p2);
	for (size_t j = 2; j < vertices_.size(); ++j) {
	  Point p3 = vertices_[(i + j) % vertices_.size()];
	  bool curr_side;
	  if (p1.x == p2.x) {
		curr_side = (p2.y > p1.y == p3.x > p1.x);
	  } else {
		curr_side = (p1.x < p2.x == p3.y < line.angle * p3.x + line.shift);
	  }
	  if (curr_side != side) {
		return false;
	  }
	}
  }
  return true;
}

double Polygon::perimeter() const {
  double result = Distance(vertices_.front(), vertices_.back());
  for (size_t i = 0; i < vertices_.size() - 1; ++i) {
	result += Distance(vertices_[i], vertices_[i + 1]);
  }
  return result;
}

double Polygon::area() const {
  double result = vertices_.back().x * vertices_.front().y - vertices_.back().y * vertices_.front().x;
  for (size_t i = 0; i < vertices_.size() - 1; ++i) {
	result += vertices_[i].x * vertices_[i + 1].y - vertices_[i].y * vertices_[i + 1].x;
  }
  result = fabs(result) / 2;
  return result;
}

bool Polygon::containsPoint(const Point& point) const {
  int intersections = 0;
  Point inf(std::numeric_limits<double>::max(), point.y);
  for (size_t i = 0; i < vertices_.size(); ++i) {
	Point p1 = vertices_[i];
	Point p2 = vertices_[(i + 1) % vertices_.size()];
	if (Polygon(p1, point, p2, inf).isConvex()) {
	  ++intersections;
	}
  }
  return intersections % 2 == 1;
}

bool Polygon::operator==(const Shape& other) const {
  const Polygon* p = dynamic_cast<const Polygon*>(&other);
  if (p == nullptr) {
	return false;
  }
  if (p->vertices_.size() != vertices_.size()) {
	return false;
  }
  for (size_t i = 0; i < vertices_.size(); ++i) {
	if (vertices_[i] == p->vertices_[0]) {
	  bool flag = true;
	  for (size_t j = 0; j < vertices_.size(); ++j) {
		if (vertices_[(i + j) % vertices_.size()] != p->vertices_[j]) {
		  flag = false;
		  break;
		}
	  }
	  if (flag) {
		return true;
	  }
	  flag = true;
	  for (size_t j = 0; j < vertices_.size(); ++j) {
		if (vertices_[(i - j + vertices_.size()) % vertices_.size()] != p->vertices_[j]) {
		  flag = false;
		  break;
		}
	  }
	  if (flag) {
		return true;
	  }
	}
  }
  return false;
}

bool Polygon::isCongruentTo(const Shape& other) const { return (isSimilarTo(other) && fabs(area() - other.area()) < 0.00001); }

bool Polygon::isSimilarTo(const Shape& other) const {
  const Polygon* p = dynamic_cast<const Polygon*>(&other);
  if (p == nullptr) {
	return false;
  }
  if (p->vertices_.size() != vertices_.size()) {
	return false;
  }
  const double kEps = 0.000000001;
  double ang = tan(Line(p->vertices_[0], p->vertices_[1]), Line(p->vertices_[1], p->vertices_[2]));
  bool flag;
  for (size_t i = 0; i < vertices_.size(); ++i) {
	Point p1 = vertices_[i];
	Point p2 = vertices_[(i + 1) % vertices_.size()];
	Point p3 = vertices_[(i + 2) % vertices_.size()];
	if (fabs(tan(Line(p1, p2), Line(p2, p3)) - ang) < kEps) {
	  flag = true;
	  for (size_t j = 0; j < vertices_.size(); ++j) {
		if (fabs(tan(Line(vertices_[(i + j) % vertices_.size()], vertices_[(i + 1 + j) % vertices_.size()]),
					 Line(vertices_[(i + 1 + j) % vertices_.size()], vertices_[(i + 2 + j) % vertices_.size()])) -
				 tan(Line(p->vertices_[j], p->vertices_[(j + 1) % vertices_.size()]),
					 Line(p->vertices_[(j + 1) % vertices_.size()], p->vertices_[(j + 2) % vertices_.size()]))) > kEps) {
		  flag = false;
		  break;
		}
	  }
	  if (flag) {
		return true;
	  }
	  flag = true;
	  for (size_t j = 0; j < vertices_.size(); ++j) {
		if (fabs(tan(Line(vertices_[(i - j + vertices_.size()) % vertices_.size()], vertices_[(i + 1 - j + vertices_.size()) % vertices_.size()]),
					 Line(vertices_[(i + 1 - j + vertices_.size()) % vertices_.size()], vertices_[(i + 2 - j + vertices_.size()) % vertices_.size()])) -
				 tan(Line(p->vertices_[j], p->vertices_[(j + 1) % vertices_.size()]),
					 Line(p->vertices_[(j + 1) % vertices_.size()], p->vertices_[(j + 2) % vertices_.size()]))) > kEps) {
		  flag = false;
		  break;
		}
	  }
	  if (flag) {
		return true;
	  }
	}
  }
  return false;
}

void Polygon::rotate(const Point& center, double angle) {
  double s = sin(angle);
  double c = cos(angle);
  for (Point& el : vertices_) {
	el.x -= center.x;
	el.y -= center.y;
	double new_x = el.x * c - el.y * s;
	double new_y = el.x * s + el.y * c;
	el.x = new_x;
	el.y = new_y;
	el.x += center.x;
	el.y += center.y;
  }
}

void Polygon::reflect(const Line& axis) {
  const double coef = 1 / (axis.angle * axis.angle + 1);
  for (Point& el : vertices_) {
	double new_x = coef * (el.x * (1 - axis.angle * axis.angle) + el.y * 2 * axis.angle - 2 * axis.angle * axis.shift);
	double new_y = coef * (el.x * 2 * axis.angle + el.y * (axis.angle * axis.angle - 1) + 2 * axis.shift);
	el.x = new_x;
	el.y = new_y;
  }
}

void Polygon::reflect(const Point& center) {
  for (Point& el : vertices_) {
	el.x -= center.x;
	el.y -= center.y;
	el.x = -el.x;
	el.y = -el.y;
	el.x += center.x;
	el.y += center.y;
  }
}

void Polygon::scale(const Point& center, double coefficient) {
  for (Point& el : vertices_) {
	double diff_x = (center.x - el.x) * coefficient;
	double diff_y = (center.y - el.y) * coefficient;
	el.x = center.x - diff_x;
	el.y = center.y - diff_y;
  }
}

class Ellipse : public Shape {
 public:
  Ellipse() = default;
  Ellipse(const Point& f1, const Point& f2, double length) : f1_(f1), f2_(f2), radius_(length * 0.5) {}
  std::pair<Point,Point> focuses() const { return std::make_pair(f1_, f2_); }
  std::pair<Line, Line> directrices() const;
  double eccentricity() const;
  Point center() const {
	return {(f1_.x + f2_.x) / 2, (f1_.y + f2_.y) / 2};
  }
  double perimeter() const override;
  double area() const override;
  bool containsPoint(const Point& point) const final;
  bool operator==(const Shape& other) const final;
  bool operator!=(const Shape& other) const final { return !(*this == other); }
  bool isCongruentTo(const Shape& other) const final;
  bool isSimilarTo(const Shape& other) const final;
  void rotate(const Point& center, double angle) final;
  void reflect(const Point& center) final;
  void reflect(const Line& axis) final;
  void scale(const Point& center, double coefficient) final;

 protected:
  Point f1_ = {0, 0};
  Point f2_ = {0, 0};
  double radius_ = 0.f;
};

double Ellipse::eccentricity() const {
  return Distance(f1_, f2_) / radius_ * 0.5;
}

std::pair<Line, Line> Ellipse::directrices() const {
  Line fokal(f1_, f2_);
  double e = eccentricity();
  double x_right = 0;
  double y_right = 0;
  double x_left = 0;
  double y_left = 0;
  if (f1_.x <= f2_.x) {
	x_right = f2_.x + (radius_ / e) / sqrt(pow(fokal.angle, 2) + 1);
	y_right = f2_.y + (radius_ / e) / sqrt(pow(1 / fokal.angle, 2) + 1);
	x_left = f1_.x - (radius_ / e) / sqrt(pow(fokal.angle, 2) + 1);
	y_left = f1_.y - (radius_ / e) / sqrt(pow(1 / fokal.angle, 2) + 1);
  } else {
	x_right = f2_.x - (radius_ / e) / sqrt(pow(fokal.angle, 2) + 1);
	y_right = f2_.y - (radius_ / e) / sqrt(pow(1 / fokal.angle, 2) + 1);
	x_left = f1_.x + (radius_ / e) / sqrt(pow(fokal.angle, 2) + 1);
	y_left = f1_.y + (radius_ / e) / sqrt(pow(1 / fokal.angle, 2) + 1);
  }
  Line a({x_left, y_left}, -1 / fokal.angle);
  Line b({x_right, y_right}, -1 / fokal.angle);
  return std::make_pair(a, b);
}

double Ellipse::perimeter() const {
  double a = radius_;
  double b = sqrt(a * a - pow(Distance(f1_, f2_) * 0.5, 2));
  double h = pow(a - b, 2) / pow(a + b, 2);
  double res = M_PI * (a + b) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
  return res;
}

double Ellipse::area() const {
  double a = radius_;
  double b = sqrt(a * a - pow(Distance(f1_, f2_) * 0.5, 2));
  return M_PI * a * b;
}

bool Ellipse::containsPoint(const Point& point) const {
  return Distance(point, f1_) + Distance(point, f2_) <= 2 * radius_;
}

bool Ellipse::operator==(const Shape& other) const {
  const Ellipse* e = dynamic_cast<const Ellipse*>(&other);
  if (e == nullptr) {
	return false;
  }
  if (radius_ != e->radius_) {
	return false;
  }
  return (f1_ == e->f1_ && f2_ == e->f2_) || (f1_ == e->f2_ && f2_ == e->f1_);
}

bool Ellipse::isCongruentTo(const Shape& other) const {
  const double kEps = 0.000000001;
  const Ellipse* e = dynamic_cast<const Ellipse*>(&other);
  if (e == nullptr) {
	return false;
  }
  return (fabs(radius_ - e->radius_) < kEps) && (fabs(Distance(f1_, f2_) - Distance(e->f1_, e->f2_)) < kEps);
}

bool Ellipse::isSimilarTo(const Shape& other) const {
  const double  kEps = 0.000000001;
  const Ellipse* e = dynamic_cast<const Ellipse*>(&other);
  if (e == nullptr) {
	return false;
  }
  return fabs(eccentricity() - e->eccentricity()) < kEps;
}

void Ellipse::rotate(const Point& center, double angle) {
  double s = sin(angle);
  double c = cos(angle);
  f1_.x -= center.x;
  f1_.y -= center.y;
  f2_.x -= center.x;
  f2_.y -= center.y;
  double new_x_1 = f1_.x * c - f1_.y * s;
  double new_y_1 = f1_.x * s + f1_.y * c;
  double new_x_2 = f2_.x * c - f2_.y * s;
  double new_y_2 = f2_.x * s + f2_.y * c;
  f1_.x = new_x_1;
  f1_.y = new_y_1;
  f2_.x = new_x_2;
  f2_.y = new_y_2;
  f1_.x += center.x;
  f1_.y += center.y;
  f2_.x += center.x;
  f2_.y += center.y;
}

void Ellipse::reflect(const Line& axis) {
  const double coef = 1 / (axis.angle * axis.angle + 1);
  double new_x_1 = coef * (f1_.x * (1 - axis.angle * axis.angle) + f1_.y * 2 * axis.angle - 2 * axis.angle * axis.shift);
  double new_y_1 = coef * (f1_.x * 2 * axis.angle + f1_.y * (axis.angle * axis.angle - 1) + 2 * axis.shift);
  double new_x_2 = coef * (f2_.x * (1 - axis.angle * axis.angle) + f2_.y * 2 * axis.angle - 2 * axis.angle * axis.shift);
  double new_y_2 = coef * (f2_.x * 2 * axis.angle + f2_.y * (axis.angle * axis.angle - 1) + 2 * axis.shift);
  f1_.x = new_x_1;
  f1_.y = new_y_1;
  f2_.x = new_x_2;
  f2_.y = new_y_2;
}

void Ellipse::reflect(const Point& center) {
  f1_.x -= center.x;
  f1_.y -= center.y;
  f2_.x -= center.x;
  f2_.y -= center.y;
  f1_.x = -f1_.x;
  f1_.y = -f1_.y;
  f2_.x = -f2_.x;
  f2_.y = -f2_.y;
  f1_.x += center.x;
  f1_.y += center.y;
  f2_.x += center.x;
  f2_.y += center.y;
}

void Ellipse::scale(const Point& center, double coefficient) {
  double diff_x_1 = (center.x - f1_.x) * coefficient;
  double diff_y_1 = (center.y - f1_.y) * coefficient;
  double diff_x_2 = (center.x - f2_.x) * coefficient;
  double diff_y_2 = (center.y - f2_.y) * coefficient;
  f1_.x = center.x - diff_x_1;
  f1_.y = center.y - diff_y_1;
  f2_.x = center.x - diff_x_2;
  f2_.y = center.y - diff_y_2;
  radius_ *= coefficient;
}

class Circle final : public Ellipse {
 public:
  Circle() = default;
  Circle(const Point& center, double radius) {
	f1_ = center;
	f2_ = center;
	radius_ = radius;
  }
  double radius() const { return radius_; }
  double perimeter() const final;
  double area() const final;
};

double Circle::perimeter() const { return 2 * M_PI * radius_; }

double Circle::area() const { return M_PI * radius_ * radius_; }

class Rectangle : public Polygon {
 public:
  Rectangle() = default;
  Rectangle(const Point& p1, const Point& p2, double ratio);
  Point center() const { return {(vertices_[0].x + vertices_[2].x) / 2, (vertices_[0].y + vertices_[2].y) / 2}; }
  std::pair<Line, Line> diagonals() const { return std::make_pair(Line(vertices_[0], vertices_[2]), Line(vertices_[1], vertices_[3])); }
};

Rectangle::Rectangle(const Point& p1, const Point& p2, double ratio) {
  double dist = Distance(p1, p2);
  Line line(p1, p2);
  double side_1 = sqrt(pow(dist, 2) / (pow(ratio, 2) + 1));
  double side_2 = sqrt(pow(dist, 2) / (pow(1 / ratio, 2) + 1));
  double tg = std::min((line.angle + std::max(ratio, 1 / ratio)) / (1 - line.angle * std::max(ratio, 1 / ratio)), std::numeric_limits<double>::max());
  double diff_x_1 = sqrt(pow(std::min(side_1, side_2), 2) / (pow(tg, 2) + 1));
  double diff_y_1 = sqrt(pow(std::min(side_1, side_2), 2) / (pow(1 / tg, 2) + 1));
  double diff_x_2 = sqrt(pow(std::max(side_1, side_2), 2) / (pow(1 / tg, 2) + 1));
  double diff_y_2 = sqrt(pow(std::max(side_1, side_2), 2) / (pow(tg, 2) + 1));
  Point p3(0, 0);
  Point p4(0, 0);
  if (tg >= 0) {
	if (p1.x <= p2.x) {
	  p3 = Point(p1.x + diff_x_1, p1.y + diff_y_1);
	  p4 = Point(p1.x + diff_x_2, p1.y - diff_y_2);
	} else {
	  p3 = Point(p1.x - diff_x_1, p1.y - diff_y_1);
	  p4 = Point(p1.x - diff_x_2, p1.y + diff_y_2);
	}
  } else {
	if (p1.x < p2.x) {
	  p3 = Point(p1.x - diff_x_1, p1.y + diff_y_1);
	  p4 = Point(p1.x + diff_x_2, p1.y + diff_y_2);
	} else if (p1.x > p2.x) {
	  p3 = Point(p1.x + diff_x_1, p1.y - diff_y_1);
	  p4 = Point(p1.x - diff_x_2, p1.y - diff_y_2);
	} else {
	  if (p1.y > p2.y) {
		p3 = Point(p1.x + diff_x_1, p1.y - diff_y_1);
		p4 = Point(p1.x - diff_x_2, p1.y - diff_y_2);
	  } else {
		p3 = Point(p1.x - diff_x_1, p1.y + diff_y_1);
		p4 = Point(p1.x + diff_x_2, p1.y + diff_y_2);
	  }
	}
  }
  vertices_.push_back(p1);
  vertices_.push_back(p3);
  vertices_.push_back(p2);
  vertices_.push_back(p4);
}

class Square final : public Rectangle {
 public:
  Square() = default;
  Square(const Point& p1, const Point& p2) : Rectangle(p1, p2, 1.f) {};
  Circle circumscribedCircle() const;
  Circle inscribedCircle() const;
};

Circle Square::circumscribedCircle() const { return {center(), Distance(center(), vertices_[0])}; }

Circle Square::inscribedCircle() const {
  return {center(), Distance(center(), Point((vertices_[0].x + vertices_[1].x) * 0.5, (vertices_[0].y + vertices_[1].y) * 0.5))};
}

class Triangle final : public Polygon {
 public:
  Triangle() = default;
  Triangle(const Point& p1, const Point& p2, const Point& p3);
  Circle circumscribedCircle() const;
  Circle inscribedCircle() const;
  Point centroid() const;
  Point orthocenter() const;
  Line EulerLine() const;
  Circle ninePointsCircle() const;
};

Triangle::Triangle(const Point& p1, const Point& p2, const Point& p3) {
  vertices_.push_back(p1);
  vertices_.push_back(p2);
  vertices_.push_back(p3);
}

Point Triangle::centroid() const {
  Point m1((vertices_[0].x + vertices_[1].x) * 0.5, (vertices_[0].y + vertices_[1].y) * 0.5);
  Point m2((vertices_[0].x + vertices_[2].x) * 0.5, (vertices_[0].y + vertices_[2].y) * 0.5);
  Line l1(vertices_[2], m1);
  Line l2(vertices_[1], m2);
  double x;
  double y;
  if (fabs(l1.angle) < std::numeric_limits<double>::max() && fabs(l2.angle) < std::numeric_limits<double>::max()) {
	x = (l1.shift - l2.shift) / (l2.angle - l1.angle);
	y = l1.angle * x + l1.shift;
  } else {
	if (fabs(l2.angle) < std::numeric_limits<double>::max()) {
	  x = vertices_[2].x;
	  y = l2.angle * x + l2.shift;
	} else {
	  x = vertices_[1].x;
	  y = l1.angle * x + l1.shift;
	}
  }
  return {x, y};
}

Point Triangle::orthocenter() const {
  Line l1(vertices_[0], vertices_[1]);
  Line l2(vertices_[1], vertices_[2]);
  Line l3(vertices_[2], -1 / l1.angle);
  Line l4(vertices_[0], -1 / l2.angle);
  double x;
  double y;
  if (fabs(l3.angle) < std::numeric_limits<double>::max() && fabs(l4.angle) < std::numeric_limits<double>::max()) {
	x = (l3.shift - l4.shift) / (l4.angle - l3.angle);
	y = l3.angle * x + l3.shift;
  } else {
	if (fabs(l4.angle) < std::numeric_limits<double>::max()) {
	  x = vertices_[2].x;
	  y = l4.angle * x + l4.shift;
	} else {
	  x = vertices_[0].x;
	  y = l3.angle * x + l3.shift;
	}
  }
  return {x, y};
}

Circle Triangle::circumscribedCircle() const {
  Line l1(vertices_[0], vertices_[1]);
  Line l2(vertices_[1], vertices_[2]);
  Line l3({(vertices_[0].x + vertices_[1].x) / 2, (vertices_[0].y + vertices_[1].y) / 2}, -1 / l1.angle);
  Line l4({(vertices_[2].x + vertices_[1].x) / 2, (vertices_[2].y + vertices_[1].y) / 2}, -1 / l2.angle);
  double x = 0;
  double y = 0;
  if (fabs(l3.angle) < std::numeric_limits<double>::max() && fabs(l4.angle) < std::numeric_limits<double>::max()) {
	x = (l3.shift - l4.shift) / (l4.angle - l3.angle);
	y = l3.angle * x + l3.shift;
  } else {
	if (fabs(l4.angle) < std::numeric_limits<double>::max()) {
	  x = (vertices_[0].x + vertices_[1].x) / 2;
	  y = l4.angle * x + l4.shift;
	} else {
	  x = (vertices_[2].x + vertices_[1].x) / 2;
	  y = l3.angle * x + l3.shift;
	}
  }
  return {{x, y}, Distance({x, y}, vertices_[0])};
}

Line Triangle::EulerLine() const { return {centroid(), orthocenter()}; }

Circle Triangle::ninePointsCircle() const {
  Triangle t({(vertices_[0].x + vertices_[1].x) / 2, (vertices_[0].y + vertices_[1].y) / 2}, {(vertices_[0].x + vertices_[2].x) / 2, (vertices_[0].y + vertices_[2].y) / 2}, {(vertices_[2].x + vertices_[1].x) / 2, (vertices_[2].y + vertices_[1].y) / 2});
  return t.circumscribedCircle();
}

Circle Triangle::inscribedCircle() const {
  double radius = Polygon::area() * 2 / Polygon::perimeter();
  double x1 = vertices_[0].x;
  double y1 = vertices_[0].y;
  double x2 = vertices_[1].x;
  double y2 = vertices_[1].y;
  double x3 = vertices_[2].x;
  double y3 = vertices_[2].y;
  double side1 = Distance(vertices_[0], vertices_[1]);
  double side2 = Distance(vertices_[1], vertices_[2]);
  double side3 = Distance(vertices_[2], vertices_[0]);
  double p = Polygon::perimeter();
  double xp = (x1 * side2 + x2 * side3 + x3 * side1) / p;
  double yp = (y1 * side2 + y2 * side3 + y3 * side1) / p;
  return {{xp, yp}, radius};
}
