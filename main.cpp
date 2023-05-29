#include <cmath>
#include <memory>
#include <map>
#include <vector>
#include <list>
#include <random>
#include <unordered_set>
#include <functional>
#include <iostream>

struct Position
{
    [[nodiscard]] float Length() const
    {
        // 距離原點長度
        return std::sqrt(x * x + y * y);
    }

    [[nodiscard]] float InnerProduct(Position position) const
    {
        // 內積
        return x * position.x + y * position.y;
    }

    // 座標 x, y
    float x, y;
};

Position operator+(const Position &lhs, const Position &rhs)
{
    // 座標相加
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Position operator-(const Position &lhs, const Position &rhs)
{
    // 座標前面減去後面
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

struct Food
{
    // 座標
    Position position;
    // 剩下的時間
    int leftTime;
};

class Snake
{
public:
    // 蛇蛇的單位大小
    static const int kSegmentSize = 20;

    Snake() = default;
    // 傳入值：蛇蛇編號、蛇蛇位置、蛇蛇方向、蛇蛇長度
    Snake(size_t id, const Position &pos, float dir, size_t len);

    void StepForward();
    void StepLeft();
    void StepRight();

    // 回傳數字型態方向
    [[nodiscard]] float Direction() const { return direction_; }
    // 頭是在 body.back() 的位置
    [[nodiscard]] const Position &Head() const { return body_.back(); }
    // 回傳整個身體
    [[nodiscard]] const std::list<Position> &Body() const { return body_; }
    // 傳入分數加分
    void AddScores(int added);

    [[nodiscard]] Position Forward() const;

    [[nodiscard]] size_t Id() const { return id_; }

    [[nodiscard]] int Scores() const { return scores_; }

private:
    float direction_ = 0;
    std::list<Position> body_;
    int scores_ = 0;
    size_t length_ = 0;
    size_t id_ = 0;
};

Snake::Snake(size_t id, const Position &pos, float dir, size_t len)
{
    length_ = len;
    id_ = id;
    direction_ = dir;
    // 加一確保不是零
    int body_size = static_cast<int>(length_) * Snake::kSegmentSize + 1;
    // 初始會在一個點，不管蛇的身體大小，然後移動後會越來越長
    for (int i = body_size; i >= 0; --i)
    {
        body_.push_back(pos);
    }
}

void Snake::StepLeft()
{
    // 數值減少：逆時針
    direction_--;
    // 轉完向後要前進
    StepForward();
}

void Snake::StepForward()
{
    // body_.back() + Forward() 是頭部的新座標 （一次移動兩單位長度）
    body_.push_back(body_.back() + Forward());
    // 蛇的尾巴去除
    body_.pop_front();
}

void Snake::StepRight()
{
    // 數值增加：順時針
    direction_++;
    // 轉完向後要前進
    StepForward();
}

void Snake::AddScores(int added)
{
    // 加分，蛇蛇長度和得分一樣
    scores_ += added;
    length_ = scores_;
    size_t body_size = length_ * Snake::kSegmentSize + 1;
    Position tail = body_.front();
    // 因分數增加的身體部分先全部加在尾巴位置
    while (body_size > body_.size())
    {
        body_.push_front(tail);
    }
}

Position Snake::Forward() const
{
    // 回傳移動向量，一次移動長度為 2
    return Position{
        static_cast<float>(2. * std::cos(direction_ / 180.f * M_PI)),
        static_cast<float>(2. * std::sin(direction_ / 180.f * M_PI))};
}

enum class DirectionType
{
    // DirectionType::kForward 是一個物件
    kForward,
    kLeft,
    kRight,
};
class Game;

struct ISnakeController {//interface for snake controller
    virtual DirectionType NextDirection(const Game& game, size_t id) = 0;
    virtual ~ISnakeController() = default;
};

class Game {
public:
    static const int kFoodRadius = 5;
    static const int kSnakeRadius = 50;
    static const int kCellSize = 100;
    static const int kMaxLeftTime = 20000;
    static const int kMinLeftTime = 2000;

    Game(size_t number_of_rows, size_t number_of_cols, size_t timeLimit);

    void RunUntilTheEnd() {//如果遊戲還沒結束就繼續執行Step
        while (!IsOver()) Step();
    }

    void AddFood(const Food& food) {//新增食物
        foods_.push_back(food);
    }

    void AddSnake(size_t id, const Position& pos, float dir, size_t len) {//新增蛇
        snakes_[id] = Snake(id, pos, dir, len);
    }

    void AddController(size_t id, const std::shared_ptr<ISnakeController>& controller) {
        controllers[id] = controller;//connected controller and snakes
    }

    void Step();

    [[nodiscard]] const Position& Center() const {
        return snakes_.at(1).Head();//return ID is 1's snake's head
    }

    template<typename UnaryFunction> void TraverseFoods(UnaryFunction f) const;
    template<typename UnaryFunction> void TraverseSnakes(UnaryFunction f) const;

    [[nodiscard]] const std::vector<Food>& Foods() const { return foods_; }
    [[nodiscard]] const std::map<size_t, Snake>& Snakes() const { return snakes_; };

    [[nodiscard]] size_t NumberOfCols() const;
    [[nodiscard]] size_t NumberOfRows() const;

    [[nodiscard]] size_t FieldWidth() const {//像素橫向(cols)數量*像素寬度
        return NumberOfCols() * kCellSize;
    }

    [[nodiscard]] size_t FieldHeight() const {
        return NumberOfRows() * kCellSize;
    }

    [[nodiscard]] bool IsOver() const;//遊戲是否結束

    [[nodiscard]] int Scores() const;//回傳分數

    [[nodiscard]] size_t Time() const;//time = ticks = 總共steps

    static bool IsCollidedWithCircle(//check if snake collides with circle 
        const Position& center_a, int radius_a,
        const Position& center_b, int radius_b);

    static bool IsCollidedWithRectangle(//check if snake collides with rectangle
        const Position& position, int radius,
        Position topLeft, Position bottomRight);

private:
    std::map<size_t, Snake> snakes_;
    std::map<size_t, std::shared_ptr<ISnakeController>> controllers;
    std::vector<Food> foods_;
    bool is_game_over_ = false;

    size_t number_of_rows_;
    size_t number_of_cols_;
    size_t timeLimit_;
    size_t ticks_;
};

template<typename Func>
void Game::TraverseFoods(Func f) const {
    for (const auto& food : foods_) {
        f(food);
    }
}
//???
template<typename Func>
void Game::TraverseSnakes(Func f) const {
    for (auto it = snakes_.begin(); it != snakes_.end(); ++it) {
        f(it->second);
    }
}


Game::Game(size_t number_of_rows, size_t number_of_cols, size_t timeLimit) :
    number_of_rows_{ number_of_rows },
    number_of_cols_{ number_of_cols },
    timeLimit_{ timeLimit },
    ticks_{ 0 } {}

void Game::Step() {
    if (is_game_over_) return;

    ++ticks_;

    if (timeLimit_ <= ticks_) {
        is_game_over_ = true;
        return;
    }

    // Move
    for (auto it = snakes_.begin(); it != snakes_.end(); ++it) {
        size_t id = it->first;
        Snake& snake = it->second;
        if (controllers.count(id)) {
            switch (controllers[id]->NextDirection(*this, id)) {
            case DirectionType::kLeft:
                snake.StepLeft();
                break;
            case DirectionType::kRight:
                snake.StepRight();
                break;
            case DirectionType::kForward:
                snake.StepForward();
                break;
            }
        }
    }

    // Dead
    std::unordered_set<size_t> dead_ids;
    for (auto it1 = snakes_.begin(); it1 != snakes_.end(); ++it1) {
        const size_t id1 = it1->first;
        const auto& snake1 = it1->second;
        const auto& forward = snake1.Forward();

        // Snake to Field
        if (IsCollidedWithRectangle(
            snake1.Head(), Game::kSnakeRadius,
            Position{ 0, 0 },
            Position{
                    static_cast<float>(FieldWidth()),
                    static_cast<float>(FieldHeight()) }
                    )) {
            dead_ids.insert(id1);
            break;
        }

        for (auto it2 = snakes_.begin(); it2 != snakes_.end(); ++it2) {
            const size_t id2 = it2->first;
            const auto& snake2 = it2->second;

            if (id1 == id2) continue;

            // Snake to Snake
            if (IsCollidedWithCircle(snake1.Head(), Game::kSnakeRadius,
                snake2.Head(), Game::kSnakeRadius)) {
                const Position distance = snake2.Head() - snake1.Head();
                const float innerProduct = distance.InnerProduct(forward);
                if (innerProduct > 0) {
                    dead_ids.insert(id1);
                }
                break;
            }
            for (const Position& pos : snake2.Body()) {
                if (IsCollidedWithCircle(snake1.Head(), Game::kSnakeRadius,
                    pos, Game::kSnakeRadius)) {
                    dead_ids.insert(id1);
                    break;
                }
            }
        }
    }

    if (!dead_ids.empty()) {
        if (dead_ids.find(1) != dead_ids.end()) {
            is_game_over_ = true;
            return;
        }

        std::map<size_t, Snake> newSnakes;
        for (auto it = snakes_.begin(); it != snakes_.end(); ++it) {
            const size_t id = it->first;
            Snake& snake = it->second;
            if (dead_ids.find(id) == dead_ids.end()) {
                newSnakes[id] = std::move(snake);
            }
            else {
                const int kPeriod = 20;
                int currIndex = 0;
                for (const auto& p : snake.Body()) {
                    if (currIndex % kPeriod == 0) {
                        foods_.push_back({ p, kMaxLeftTime });
                    }
                    currIndex++;
                }
            }
        }
        snakes_ = std::move(newSnakes);
    }

    // Eat
    std::vector<Food> new_foods;
    for (auto& food : foods_) {
        bool is_eaten = false;
        for (auto it = snakes_.begin(); it != snakes_.end(); ++it) {
            Snake& snake = it->second;
            if (IsCollidedWithCircle(
                snake.Head(), Game::kSnakeRadius,
                food.position, Game::kFoodRadius)) {
                snake.AddScores(1);
                is_eaten = true;
                break;
            }
        }
        if (!is_eaten) {
            food.leftTime--;
            if (food.leftTime > 0) {
                new_foods.push_back(food);
            }
        }
    }
    std::swap(new_foods, foods_);
}

bool Game::IsCollidedWithCircle(
    const Position& center_a,
    int radius_a,
    const Position& center_b,
    int radius_b) {
    return ((center_a - center_b).Length() < static_cast<float>(radius_a + radius_b));
}

size_t Game::NumberOfCols() const {
    return number_of_cols_;
}

size_t Game::NumberOfRows() const {
    return number_of_rows_;
}

bool Game::IsOver() const {
    return is_game_over_;
}

bool Game::IsCollidedWithRectangle(
    const Position& position, const int radius,
    Position topLeft, Position bottomRight) {
    auto fRadius = static_cast<float>(radius);
    return
        position.x - fRadius < topLeft.x ||
        position.x + fRadius > bottomRight.x ||
        position.y - fRadius < topLeft.y ||
        position.y + fRadius > bottomRight.y;
}

int Game::Scores() const {
    return snakes_.at(1).Scores();
}

size_t Game::Time() const {
    return ticks_;
}

// 朝同一個方向一直走
class ConstantDirectionControllerA : public ISnakeController {
public:
    explicit ConstantDirectionControllerA(DirectionType type) : type_{type} {}
    DirectionType NextDirection(const Game&, size_t) override {
        return type_;
    }
private:
    DirectionType type_;
};

// 繞圈圈
class ConstantDirectionControllerB : public ISnakeController {
public:
    DirectionType NextDirection(const Game&, size_t id) override {
		if (type_ == DirectionType::kForward) {
			type_ = DirectionType::kRight;
		} else {
			type_ = DirectionType::kForward;
		}
        return type_;
    }
private:
    DirectionType type_ = DirectionType::kRight;
};

class StraightForwardController : public ISnakeController {
public:
    explicit StraightForwardController(float init_dir, DirectionType turnDirection) :
        _final_angle { init_dir },
        _turnDirection { turnDirection },
        _currentDirType { DirectionType::kForward },
        _dirSymbol { AngleToSymbol(_final_angle) } {} // 
    DirectionType NextDirection(const Game&, size_t) override;
private:
    enum class DirectionSymbol {
        RIGHT, DOWN, LEFT, UP, NONE
    };
    DirectionType _turnDirection;   // 旋轉後方向(Symbol)
    DirectionType _currentDirType;  // 目前方向(Symbol)
    float _final_angle;             // 最後方向(Angle)
    DirectionSymbol _dirSymbol;
    const float turn_radius =  3 * 180 / 3.1415926 + 30;

    DirectionSymbol AngleToSymbol(float);
    float GetCollisionDistance(Position, DirectionSymbol, const Game&, size_t);
    float GetFrontCollisionDistance(Position, float, DirectionSymbol, Position, float);
    float FrontWallDistance(Position, DirectionSymbol, float, float);
};

// 如果不會撞上就一直往前走，如果會撞上就左轉或右轉
DirectionType StraightForwardController::NextDirection(const Game& game, size_t id) {
    const auto& snake = game.Snakes().at(id); // 根據提供的 id 從遊戲中獲取相對應的蛇

    // 若移動方向不是直線前進 
    if (_currentDirType != DirectionType::kForward) { 
        float remaining_angle = abs(_final_angle - snake.Direction()); 

        // 若最後方向與目前方向不同，則持續旋轉
        if (remaining_angle > 0) {
            return _currentDirType;
        }

        // 若最後方向與目前方向相同，取得旋轉前方向(Symbol)
        _dirSymbol = AngleToSymbol(snake.Direction());
    }

    float distance = GetCollisionDistance(snake.Head(), _dirSymbol, game, id);

    // 若在下一步移動中會發生碰撞(if 0 < distance < min_distance)
    if (distance > 0 && distance < turn_radius) {

        // 更新目前方向成旋轉後方向(Symbol)
        _currentDirType = _turnDirection;

        // 若旋轉後方向是Right，代表下一步往右走會撞到，要更新成左轉(+180)
        if (_currentDirType == DirectionType::kRight) {
            _final_angle = snake.Direction() + 180;
        }

        // 若旋轉後方向是Left，代表下一步往左走會撞到，要更新成右轉(-180)
        else {
            _final_angle = snake.Direction() - 180;
        }
        
    }

    // 若沒有碰撞問題就一直直走
    else {
        _currentDirType = DirectionType::kForward;
    }
    return _currentDirType;
}

float StraightForwardController::GetCollisionDistance(Position snakePos, DirectionSymbol dirSymbol, const Game& game, size_t id) {
    
    // check front collision distance with field
    float distance = FrontWallDistance(snakePos, dirSymbol, game.FieldWidth(), game.FieldHeight());

    // check front collision distance with other snakes
    for (auto it = game.Snakes().begin(); it != game.Snakes().end(); ++it) {
        const size_t anotherID = it->first;
        const Snake& anotherSnake = it->second;
        if (anotherID == id) continue;

        float d = GetFrontCollisionDistance(snakePos, Game::kSnakeRadius * 2, dirSymbol, anotherSnake.Head(), Game::kSnakeRadius * 2);
        
        if (d > 0) {
            if (distance < 0)    distance = d;
            else {
                distance = std::min(distance, d);
            }
        }
        for (const Position& pos : anotherSnake.Body()) {
            float d_body = GetFrontCollisionDistance(snakePos, Game::kSnakeRadius, dirSymbol, pos, Game::kSnakeRadius);
            if (d_body > 0) {
                if (distance < 0)    distance = d_body;
                else {
                    distance = std::min(distance, d_body);
                }
            }
        }
    }
    return distance;
}

StraightForwardController::DirectionSymbol StraightForwardController::AngleToSymbol(float angle) {
    // if angle is not a multiple of 90
    if (int(angle) % 90 != 0) {
        return DirectionSymbol::NONE;
    }
    // can be converted into 4 directions
    int dir = abs(angle / 90);
    dir %= 4;
    return static_cast<DirectionSymbol>(dir); // RIGHT, DOWN, LEFT, UP, NONE
}
float StraightForwardController::GetFrontCollisionDistance(Position snakePos, float snakeRadius, DirectionSymbol dirSymbol, Position target, float targetRadius) {
    float distanceX = abs(snakePos.x - target.x) - snakeRadius - targetRadius;
    float distanceY = abs(snakePos.y - target.y) - snakeRadius - targetRadius;
    
    // if direction is Left/Right
    if (dirSymbol == DirectionSymbol::LEFT || dirSymbol == DirectionSymbol::RIGHT) {
        if (distanceY > 0) { // if will not hit target y, return -1
            return -1;
        }
        return distanceX;
    }

    // if direction is Up/Down
    if (dirSymbol == DirectionSymbol::UP || dirSymbol == DirectionSymbol::DOWN) {
        if (distanceX > 0) { // if will not hit target x, return -1
            return -1;
        }
        
        return distanceY;
    }

    return -1;
}
float StraightForwardController::FrontWallDistance(Position snakeHead, DirectionSymbol dirSymbol, float rightWall, float downWall) {
    Position frontFieldCollisionPos{ 0, 0 };
    if (dirSymbol == DirectionSymbol::LEFT) {
        frontFieldCollisionPos.x = 0;
        frontFieldCollisionPos.y = snakeHead.y;
    }
    else if (dirSymbol == DirectionSymbol::RIGHT) {
        frontFieldCollisionPos.x = rightWall;
        frontFieldCollisionPos.y = snakeHead.y;
    }
    else if (dirSymbol == DirectionSymbol::UP) {
        frontFieldCollisionPos.x = snakeHead.x;
        frontFieldCollisionPos.y = 0;
    }
    else if (dirSymbol == DirectionSymbol::DOWN) {
        frontFieldCollisionPos.x = snakeHead.x;
        frontFieldCollisionPos.y = downWall;
    }
    
    return GetFrontCollisionDistance(snakeHead, Game::kSnakeRadius, dirSymbol, frontFieldCollisionPos, 0);
}

// [YOUR CODE WILL BE PLACED HERE]
class CustomController : public ISnakeController
{
public:
    DirectionType NextDirection(const Game &game, size_t id);
    ~CustomController() = default;

private:
};

DirectionType CustomController::NextDirection(const Game &game, size_t id)
{
    // 確認該場有沒有其他蛇

    DirectionType whereToGo;

    // 會不會撞到邊界

    // 躲避其他蛇
    for (auto it = game.Snakes().begin(); it != game.Snakes().end(); ++it)
    {
    }
    // 找食物
    for (auto it = game.Foods().begin(); it != game.Foods().end(); ++it)
    {
    }

    return whereToGo;
}
// [YOUR CODE WILL BE PLACED HERE]

Position CreateSafePosition(const Game& game, std::mt19937& rand, size_t margin) {
    const auto space = Game::kSnakeRadius * margin;
    return
        Position{
                static_cast<float>(rand() % (game.FieldWidth() - 2 * space) + space),
                static_cast<float>(rand() % (game.FieldHeight() - 2 * space) + space),
        };
}

int CreateValidLeftTime(std::mt19937& rand) {
    return
        static_cast<int>(
                rand() % (Game::kMaxLeftTime - Game::kMinLeftTime+1) + Game::kMinLeftTime);
}

Food CreateFood(const Game& game, std::mt19937& rand) {
    auto pos = CreateSafePosition(game, rand, 2);
    auto leftTime = CreateValidLeftTime(rand);
    return {pos, leftTime};
}
Position CreateSafeSnakePosition(const Game& game, std::mt19937& rand) {
    return CreateSafePosition(game, rand, 5);
}

void TestA1(); 
void TestA2();

void TestB1();
void TestB2();

void TestC1();
void TestC2(); 

int main() {
    int id;
    std::cin >> id;
    void (*f[])() = { TestA1, TestA2, TestB1, TestB2, TestC1, TestC2 };
    f[id-1]();
}
void RunTest(std::function<Game(std::shared_ptr<ISnakeController>)> setup) {
    Game game = setup(std::make_shared<CustomController>());
    game.RunUntilTheEnd();
    std::cout << "Scores: " << game.Scores() << std::endl;
}

void TestA(unsigned seed) {
    RunTest(
            [&](const std::shared_ptr<ISnakeController> &controller) {
                Game game(25, 25, Game::kMaxLeftTime);

                std::mt19937 rand{seed};

                for (int i = 1; i <= 500; ++i) {
                    game.AddFood(CreateFood(game, rand));
                }

                game.AddSnake(1, CreateSafeSnakePosition(game, rand), 0, 10);
                game.AddController(1, controller);
                return game;
            });
}
void TestB(unsigned seed) {
    RunTest(
            [&](const std::shared_ptr<ISnakeController> &controller) {
		 		Game game(25, 25, Game::kMaxLeftTime); 
		        std::mt19937 rand{seed};
		        
		        const auto margin = Game::kSnakeRadius * 5;
		        float max_x = game.FieldWidth() - margin;
		        float max_y = game.FieldHeight() - margin;
		        float min_x = 0 + margin;
		        float min_y = 0 + margin;
		        float center_x = 0 + game.FieldWidth() / 2;
		        float center_y = 0 + game.FieldHeight() / 2;
		        Position pos{0, 0};
		
		        for (int i = 1; i <= 500; ++i) {
		            game.AddFood(CreateFood(game, rand));
		        }
		
		        game.AddSnake(1, CreateSafeSnakePosition(game, rand), 0, 10);
		        game.AddController(1, controller);
		
				pos.x = min_x + 100;
		        pos.y = min_y + 100;
		        game.AddSnake(2, pos, 0, 15);
		        game.AddController(
			        2,
			        std::make_shared<ConstantDirectionControllerB>());
                
                pos.x = min_x + 200;
		        pos.y = max_y;
		        game.AddSnake(3, pos, 0, 15);
		        game.AddController(
			        3,
			        std::make_shared<ConstantDirectionControllerA>(DirectionType::kLeft));
		
				pos.x = max_x - 150;
		        pos.y = min_y + 150;
		        game.AddSnake(4, pos, 0, 10);
		        game.AddController(
			        4,
			        std::make_shared<ConstantDirectionControllerA>(DirectionType::kRight));
		
				pos.x = max_x;
		        pos.y = max_y;
		        game.AddSnake(5, pos, 0, 15);
		        game.AddController(
			        5,
			        std::make_shared<ConstantDirectionControllerA>(DirectionType::kLeft));
		
				pos.x = center_x;
		        pos.y = center_y;
		        game.AddSnake(6, pos, 0, 25);
		        game.AddController(
			        6,
			        std::make_shared<ConstantDirectionControllerB>());
		
		        return game;
            });
}

void TestC(unsigned seed) {
    RunTest(
            [&](const std::shared_ptr<ISnakeController> &controller) {
		 		Game game(25, 25, Game::kMaxLeftTime);
		        std::mt19937 rand{ seed };
		
		        for (int i = 1; i <= 500; ++i) {
		            game.AddFood(CreateFood(game, rand));
		        }
		
		        game.AddSnake(1, CreateSafeSnakePosition(game, rand), 0, 10);
		        game.AddController(1, controller);
		
		        const auto margin = Game::kSnakeRadius * 7;
		        float max_x = game.FieldWidth() - margin;
		        float max_y = game.FieldHeight() - margin;
		        float min_x = 0 + margin;
		        float min_y = 0 + margin;
		
		        Position pos{ 0, 0 };
		        // Top Left - Go right
		        pos.x = min_x + (rand() % 8) * 50; // random shift right
		        pos.y = min_y;
		
        		game.AddSnake(2, pos, 0, 15);
		        game.AddController(
		            2,
		            std::make_shared<StraightForwardController>(0, DirectionType::kLeft));
		
		        // Bottom Left - Go up
		        pos.x = min_x;
		        pos.y = max_y - (rand() % 5) * 50; // random shift up
		
		        game.AddSnake(3, pos, 270, 15);
		        game.AddController(
		            3,
		            std::make_shared<StraightForwardController>(270, DirectionType::kRight));
		
		        // Bottom Right - Go left
		        pos.x = max_x - (rand() % 9) * 30; // random shift left
		        pos.y = max_y;
		
		        game.AddSnake(4, pos, 180, 15);
		        game.AddController(
		            4,
		            std::make_shared<StraightForwardController>(180, DirectionType::kLeft));
                    		        // Top Right - Go down
		        pos.x = max_x;
		        pos.y = min_y + (rand() % 4) * 60; // random shift down
		
		        game.AddSnake(5, pos, 90, 15);
		        game.AddController(
		            5,
		            std::make_shared<StraightForwardController>(90, DirectionType::kRight));
		
		        return game;
            });
}

void TestA1() { TestA(20); }
void TestA2() { TestA(/* HIDDEN */ 20); }

void TestB1() { TestB(25); }
void TestB2() { TestB(/* HIDDEN */ 25); }

void TestC1() { TestC(25); }
void TestC2() { TestC(/* HIDDEN */ 25); }

