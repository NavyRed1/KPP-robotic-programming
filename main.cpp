#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

enum class Dir { UP, DOWN, LEFT, RIGHT };

using NodeEntry = std::pair<int, int>;

struct Pos {
    int rowIdx;
    int ColIdx;
};

class Compass {
public:
    static Pos methodShift(Pos origin_pos, Dir moveDir) {
        Pos ShiftedPos = origin_pos;
        if (moveDir == Dir::UP) ShiftedPos.rowIdx -= 1;
        else if (moveDir == Dir::DOWN) ShiftedPos.rowIdx += 1;
        else if (moveDir == Dir::LEFT) ShiftedPos.ColIdx -= 1;
        else ShiftedPos.ColIdx += 1;
        return ShiftedPos;
    }

    static int methodDistance(Pos first_pos, Pos secondPos) {
        return std::abs(first_pos.rowIdx - secondPos.rowIdx) + std::abs(first_pos.ColIdx - secondPos.ColIdx);
    }

    static const char* methodLabel(Dir LabelDir) {
        if (LabelDir == Dir::UP) return "UP";
        if (LabelDir == Dir::DOWN) return "DOWN";
        if (LabelDir == Dir::LEFT) return "LEFT";
        return "RIGHT";
    }

    static bool methodSame(Pos left_pos, Pos rightPos) {
        return left_pos.rowIdx == rightPos.rowIdx && left_pos.ColIdx == rightPos.ColIdx;
    }

    static std::string methodFormat(Pos ShownPos) {
        return "(" + std::to_string(ShownPos.rowIdx) + "," + std::to_string(ShownPos.ColIdx) + ")";
    }

    static std::string methodJoin(const std::vector<Dir>& route_list) {
        std::string joinedText;
        for (std::size_t ItemIndex = 0; ItemIndex < route_list.size(); ++ItemIndex) {
            if (ItemIndex > 0) joinedText += ", ";
            joinedText += methodLabel(route_list[ItemIndex]);
        }
        return joinedText;
    }
};

class MazeBase {
public:
    virtual ~MazeBase() = default;
    virtual int methodSideLength() const = 0;
    virtual char methodCellAt(int, int) const = 0;

    bool methodInside(int probe_row, int probeCol) const {
        return probe_row >= 0 && probeCol >= 0 && probe_row < methodSideLength() && probeCol < methodSideLength();
    }

    bool methodWalkable(int TestRow, int test_col) const {
        if (!methodInside(TestRow, test_col)) return false;
        char tileSymbol = methodCellAt(TestRow, test_col);
        return tileSymbol != '#' && tileSymbol != 'X';
    }

    Pos methodLocate(char TargetSymbol) const {
        int side_total = methodSideLength();
        for (int rowScan = 0; rowScan < side_total; ++rowScan)
            for (int ColScan = 0; ColScan < side_total; ++ColScan)
                if (methodCellAt(rowScan, ColScan) == TargetSymbol) return Pos{rowScan, ColScan};
        return Pos{-1, -1};
    }
};

template <int grid_dim>
class Maze : public MazeBase {
    char mazeGridBuffer[grid_dim][grid_dim];

public:
    Maze(const char* const (&SourceRows)[grid_dim]) {
        for (int copy_row = 0; copy_row < grid_dim; ++copy_row)
            for (int copyCol = 0; copyCol < grid_dim; ++copyCol)
                mazeGridBuffer[copy_row][copyCol] = SourceRows[copy_row][copyCol];
    }

    int methodSideLength() const override { return grid_dim; }

    char methodCellAt(int QueryRow, int query_col) const override {
        return mazeGridBuffer[QueryRow][query_col];
    }
};

class Pathfinder {
public:
    virtual ~Pathfinder() = default;
    virtual std::vector<Dir> methodFindPath(const MazeBase&, Pos, Pos) const = 0;
};

class AStarPathfinder : public Pathfinder {
public:
    std::vector<Dir> methodFindPath(const MazeBase& mazeRef, Pos StartPos, Pos goal_pos) const override {
        int sideTotal = mazeRef.methodSideLength();
        std::vector<int> CostSoFar(sideTotal * sideTotal, -1);
        std::priority_queue<NodeEntry, std::vector<NodeEntry>, std::greater<NodeEntry>> q_traversalNode;
        std::vector<int> parentCell(sideTotal * sideTotal, -1);
        std::vector<Dir> ParentMove(sideTotal * sideTotal, Dir::UP);
        int start_id = StartPos.rowIdx * sideTotal + StartPos.ColIdx;
        int goalId = goal_pos.rowIdx * sideTotal + goal_pos.ColIdx;
        CostSoFar[start_id] = 0;
        q_traversalNode.push({Compass::methodDistance(StartPos, goal_pos), start_id});
        while (!q_traversalNode.empty()) {
            int CurrentId = q_traversalNode.top().second;
            q_traversalNode.pop();
            if (CurrentId == goalId) break;
            Pos current_pos{CurrentId / sideTotal, CurrentId % sideTotal};
            for (int dirIndex = 0; dirIndex < 4; ++dirIndex) {
                Dir NextDir = static_cast<Dir>(dirIndex);
                Pos next_pos = Compass::methodShift(current_pos, NextDir);
                if (!mazeRef.methodWalkable(next_pos.rowIdx, next_pos.ColIdx)) continue;
                int nextId = next_pos.rowIdx * sideTotal + next_pos.ColIdx;
                int NewCost = CostSoFar[CurrentId] + 1;
                if (CostSoFar[nextId] != -1 && CostSoFar[nextId] <= NewCost) continue;
                CostSoFar[nextId] = NewCost;
                parentCell[nextId] = CurrentId;
                ParentMove[nextId] = NextDir;
                q_traversalNode.push({NewCost + Compass::methodDistance(next_pos, goal_pos), nextId});
            }
        }
        std::vector<Dir> route_out;
        if (CostSoFar[goalId] == -1) return route_out;
        for (int traceId = goalId; traceId != start_id; traceId = parentCell[traceId])
            route_out.push_back(ParentMove[traceId]);
        std::reverse(route_out.begin(), route_out.end());
        return route_out;
    }
};

void Viz(const MazeBase& GridView, Pos robot_at, bool flagTaken, bool UseColor) {
    int side_count = GridView.methodSideLength();
    for (int drawRow = 0; drawRow < side_count; ++drawRow) {
        for (int DrawCol = 0; DrawCol < side_count; ++DrawCol) {
            char tile_char = GridView.methodCellAt(drawRow, DrawCol);
            if (drawRow == robot_at.rowIdx && DrawCol == robot_at.ColIdx) tile_char = 'R';
            else if (tile_char == 'F' && flagTaken) tile_char = '.';
            const char* paintCode = "";
            if (UseColor) {
                if (tile_char == 'R') paintCode = "\033[1;92m";
                else if (tile_char == '#') paintCode = "\033[90m";
                else if (tile_char == 'S') paintCode = "\033[96m";
                else if (tile_char == 'F') paintCode = "\033[93m";
                else if (tile_char == 'G') paintCode = "\033[94m";
                else if (tile_char == 'X') paintCode = "\033[91m";
            }
            std::cout << paintCode << tile_char << (UseColor ? "\033[0m" : "");
            if (DrawCol + 1 < side_count) std::cout << ' ';
        }
        std::cout << '\n';
    }
}

struct RunOptions {
    bool ShowSteps;
    bool color_on;
    int delayMs;
};

class Robot {
protected:
    std::string RobotName;
    const MazeBase& map_ref;
    Pos positionNow;
    bool FlagHeld;
    int step_total;

public:
    Robot(const std::string& nameText, const MazeBase& MapInput)
        : RobotName(nameText), map_ref(MapInput), positionNow(MapInput.methodLocate('S')), FlagHeld(false), step_total(0) {}

    bool methodMove(Dir step_dir) {
        Pos nextPos = Compass::methodShift(positionNow, step_dir);
        if (!map_ref.methodWalkable(nextPos.rowIdx, nextPos.ColIdx)) return false;
        positionNow = nextPos;
        ++step_total;
        if (map_ref.methodCellAt(nextPos.rowIdx, nextPos.ColIdx) == 'F') FlagHeld = true;
        return true;
    }

    Pos methodPosition() const { return positionNow; }
    bool methodHasFlag() const { return FlagHeld; }
    int methodStepTotal() const { return step_total; }
    const std::string& methodName() const { return RobotName; }
};

class AutoRobot : public Robot {
    const Pathfinder& PlannerRef;

public:
    AutoRobot(const std::string& auto_name, const MazeBase& autoMap, const Pathfinder& PlannerInput)
        : Robot(auto_name, autoMap), PlannerRef(PlannerInput) {}

    void methodFollow(const std::vector<Dir>& route_plan, const RunOptions& optionSet) {
        for (std::size_t PlanIndex = 0; PlanIndex < route_plan.size(); ++PlanIndex) {
            methodMove(route_plan[PlanIndex]);
            if (!optionSet.ShowSteps) continue;
            std::cout << "STEP " << step_total << " - MOVE " << Compass::methodLabel(route_plan[PlanIndex])
                      << " - POSITION " << Compass::methodFormat(positionNow) << '\n';
            Viz(map_ref, positionNow, FlagHeld, optionSet.color_on);
            std::cout << '\n';
            if (optionSet.delayMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(optionSet.delayMs));
        }
    }

    bool methodExecuteMission(const RunOptions& mission_options) {
        Pos startCell = positionNow;
        Pos FlagCell = map_ref.methodLocate('F');
        Pos goal_cell = map_ref.methodLocate('G');
        if (startCell.rowIdx < 0 || FlagCell.rowIdx < 0 || goal_cell.rowIdx < 0) {
            std::cout << "MISSION FAILED : MAP MUST CONTAIN ONE S, ONE F, AND ONE G\n";
            return false;
        }
        int sideTotal = map_ref.methodSideLength();
        std::vector<Dir> PathToFlag = PlannerRef.methodFindPath(map_ref, startCell, FlagCell);
        std::vector<Dir> path_to_base = PlannerRef.methodFindPath(map_ref, FlagCell, goal_cell);
        if (PathToFlag.empty() || path_to_base.empty()) {
            std::cout << "MISSION FAILED : NO SAFE PATH FOUND\n";
            return false;
        }
        std::cout << "ROBOT          : " << RobotName << '\n';
        std::cout << "MAP LOADED     : " << sideTotal << " x " << sideTotal << '\n';
        std::cout << "START POSITION : " << Compass::methodFormat(startCell) << '\n';
        std::cout << "FLAG POSITION  : " << Compass::methodFormat(FlagCell) << '\n';
        std::cout << "BASE POSITION  : " << Compass::methodFormat(goal_cell) << '\n';
        std::cout << "PATH TO FLAG   : " << Compass::methodJoin(PathToFlag) << '\n';
        if (mission_options.ShowSteps) {
            std::cout << "\nINITIAL MAP\n";
            Viz(map_ref, positionNow, FlagHeld, mission_options.color_on);
            std::cout << '\n';
        }
        methodFollow(PathToFlag, mission_options);
        if (!FlagHeld || !Compass::methodSame(positionNow, FlagCell)) {
            std::cout << "MISSION FAILED : FLAG NOT CAPTURED\n";
            return false;
        }
        std::cout << "FLAG CAPTURED  : " << Compass::methodFormat(positionNow) << '\n';
        std::cout << "PATH TO BASE   : " << Compass::methodJoin(path_to_base) << '\n';
        if (mission_options.ShowSteps) std::cout << '\n';
        methodFollow(path_to_base, mission_options);
        if (!Compass::methodSame(positionNow, goal_cell)) {
            std::cout << "MISSION FAILED : BASE NOT REACHED\n";
            return false;
        }
        std::cout << "BASE REACHED   : " << Compass::methodFormat(positionNow) << '\n';
        std::cout << "MISSION COMPLETE\n";
        std::cout << "TOTAL MOVES    : " << step_total << '\n';
        return true;
    }
};

int main(int argumentTotal, char* ArgList[]) {
    RunOptions picked_options{true, false, 0};
    int levelChoice = 0;
    for (int ArgIndex = 1; ArgIndex < argumentTotal; ++ArgIndex) {
        std::string arg_text = ArgList[ArgIndex];
        if (arg_text == "--summary") picked_options.ShowSteps = false;
        else if (arg_text == "--color") picked_options.color_on = true;
        else if (arg_text == "--animate") picked_options.delayMs = 300;
        else if (arg_text == "1" || arg_text == "2" || arg_text == "3") levelChoice = arg_text[0] - '0';
    }

    Maze<7> levelOne({
        "#######",
        "#S#...#",
        "#.#.#G#",
        "#...#.#",
        "###...#",
        "#X..F.#",
        "#######"});

    Maze<9> LevelTwo({
        "#########",
        "#S..#...#",
        "#.#.#.#G#",
        "#.#...#.#",
        "#...#...#",
        "###.#..##",
        "#X..#..F#",
        "#.###X..#",
        "#########"});

    Maze<11> level_three({
        "###########",
        "#S..#.....#",
        "#.#.#.###.#",
        "#.#...#G#.#",
        "#...#.#.#.#",
        "###.#...#.#",
        "#...###.#.#",
        "#.#..X..#.#",
        "#.#.###.#F#",
        "#X....X...#",
        "###########"});

    const MazeBase* levelList[3] = {&levelOne, &LevelTwo, &level_three};
    AStarPathfinder PlannerCore;
    bool all_clear = true;

    for (int runIndex = 0; runIndex < 3; ++runIndex) {
        if (levelChoice != 0 && levelChoice != runIndex + 1) continue;
        std::cout << "=== SOAL " << runIndex + 1 << " ===\n";
        AutoRobot AtlasUnit("Atlas", *levelList[runIndex], PlannerCore);
        if (!AtlasUnit.methodExecuteMission(picked_options)) all_clear = false;
        std::cout << '\n';
    }
    return all_clear ? 0 : 1;
}
