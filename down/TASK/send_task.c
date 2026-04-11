#include "send_task.h"
#include "usart.h"
#include "myusart.h"
extern u8 anjian_temp;
extern u8 error_geshu;

extern u8 error_data[100];
extern char key_confirm;
extern u8 start_flag;

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

// 定义常量
#define ROWS 7
#define COLS 9
#define MAX_PATH_POINTS 100
#define END_MARKER_ROW 9
#define END_MARKER_COL 2

// 全局变量
int g_map[ROWS][COLS];                // 地图数组：1-可飞，0-禁飞
bool g_visited[ROWS][COLS];           // 访问标记数组
int g_path[2][MAX_PATH_POINTS];       // 路径数组：[0][i]存储行，[1][i]存储列
int g_path_index = 0;                 // 当前路径索引
int g_flyable_count = 0;              // 可飞区域总数
int g_visited_count = 0;              // 已访问的可飞区域数
int g_final_path[2][MAX_PATH_POINTS]; // 最终路径
int g_final_path_length = 0;          // 最终路径长度

// 四个方向：上、右、下、左
const int dx[4] = {-1, 0, 1, 0};
const int dy[4] = {0, 1, 0, -1};

// 函数声明
bool isValid(int x, int y);
void countFlyableAreas(void);
void initPath(void);
void saveFinalPath(void);
bool findNearestUnvisited(int x, int y, int *target_x, int *target_y);
bool dfs(int x, int y);
int planPath(int map[ROWS][COLS], int path[2][MAX_PATH_POINTS]);
bool findReturnPath(int start_x, int start_y, int end_x, int end_y,
                    int return_path[2][MAX_PATH_POINTS], int *return_length);
int planPathWithReturn(int map[ROWS][COLS], int path[2][MAX_PATH_POINTS], bool include_return);
void outputPathWithReturn(int path[2][MAX_PATH_POINTS], int length);
void printMap(int map[ROWS][COLS]);

/**
 * @brief 检查坐标是否有效且可飞
 * @param x 行坐标
 * @param y 列坐标
 * @return true-有效且可飞，false-无效或禁飞
 */
bool isValid(int x, int y)
{
    if (x < 0 || x >= ROWS || y < 0 || y >= COLS)
    {
        return false;
    }
    return g_map[x][y] == 1;
}

/**
 * @brief 统计可飞区域总数
 */
void countFlyableAreas(void)
{
    g_flyable_count = 0;
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            if (g_map[i][j] == 1)
            {
                g_flyable_count++;
            }
        }
    }
}

/**
 * @brief 初始化路径数组
 */
void initPath(void)
{
    for (int i = 0; i < MAX_PATH_POINTS; i++)
    {
        g_path[0][i] = -1;
        g_path[1][i] = -1;
        g_final_path[0][i] = -1;
        g_final_path[1][i] = -1;
    }
}

/**
 * @brief 保存当前路径为最终路径
 */
void saveFinalPath(void)
{
    g_final_path_length = g_path_index;
    for (int i = 0; i < g_path_index && i < MAX_PATH_POINTS; i++)
    {
        g_final_path[0][i] = g_path[0][i];
        g_final_path[1][i] = g_path[1][i];
    }
}

/**
 * @brief 寻找最近的未访问格子
 * @param x 当前行坐标
 * @param y 当前列坐标
 * @param target_x 输出目标行坐标
 * @param target_y 输出目标列坐标
 * @return true-找到未访问格子，false-所有格子都已访问
 */
bool findNearestUnvisited(int x, int y, int *target_x, int *target_y)
{
    // 使用BFS寻找最近的未访问格子
    bool bfs_visited[ROWS][COLS];
    memset(bfs_visited, false, sizeof(bfs_visited));

    // 队列用于BFS
    int queue_x[ROWS * COLS];
    int queue_y[ROWS * COLS];
    int queue_parent_x[ROWS * COLS];
    int queue_parent_y[ROWS * COLS];
    int front = 0, rear = 0;

    // 起始点入队
    queue_x[rear] = x;
    queue_y[rear] = y;
    queue_parent_x[rear] = x;
    queue_parent_y[rear] = y;
    rear++;
    bfs_visited[x][y] = true;

    while (front < rear)
    {
        int cx = queue_x[front];
        int cy = queue_y[front];
        int px = queue_parent_x[front];
        int py = queue_parent_y[front];
        front++;

        // 检查四个方向
        for (int i = 0; i < 4; i++)
        {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (isValid(nx, ny) && !bfs_visited[nx][ny])
            {
                // 如果找到未访问的格子
                if (!g_visited[nx][ny])
                {
                    // 找到了！现在回溯找到第一步
                    if (cx == x && cy == y)
                    {
                        *target_x = nx;
                        *target_y = ny;
                    }
                    else
                    {
                        *target_x = px;
                        *target_y = py;
                    }
                    return true;
                }

                // 继续搜索
                bfs_visited[nx][ny] = true;
                queue_x[rear] = nx;
                queue_y[rear] = ny;
                // 记录从原点出发的第一步
                if (cx == x && cy == y)
                {
                    queue_parent_x[rear] = nx;
                    queue_parent_y[rear] = ny;
                }
                else
                {
                    queue_parent_x[rear] = px;
                    queue_parent_y[rear] = py;
                }
                rear++;
            }
        }
    }

    return false;
}

/**
 * @brief 深度优先搜索遍历所有可飞区域
 * @param x 当前行坐标
 * @param y 当前列坐标
 * @return true-成功遍历所有区域，false-还有未访问区域
 */
bool dfs(int x, int y)
{
    // 边界检查
    if (!isValid(x, y) || g_path_index >= MAX_PATH_POINTS - 1)
    {
        return false;
    }

    // 记录当前位置到路径
    g_path[0][g_path_index] = x;
    g_path[1][g_path_index] = y;
    g_path_index++;

    // 标记已访问
    if (!g_visited[x][y])
    {
        g_visited[x][y] = true;
        g_visited_count++;
    }

    // 检查是否已访问所有可飞区域
    if (g_visited_count == g_flyable_count)
    {
        saveFinalPath();
        return true;
    }

    // 优先访问未访问的邻居
    bool found_unvisited = false;
    for (int i = 0; i < 4; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (isValid(nx, ny) && !g_visited[nx][ny])
        {
            found_unvisited = true;
            if (dfs(nx, ny))
            {
                return true;
            }
        }
    }

    // 如果没有未访问的邻居，使用BFS找到最近的未访问格子
    if (!found_unvisited && g_visited_count < g_flyable_count)
    {
        int target_x, target_y;
        if (findNearestUnvisited(x, y, &target_x, &target_y))
        {
            // 找到了通往未访问区域的下一步
            if (dfs(target_x, target_y))
            {
                return true;
            }
        }
    }

    // 回溯（仅在无法继续时）
    g_path_index--;
    return false;
}

/**
 * @brief 主路径规划函数
 * @param map 输入的地图数组（7x9）
 * @param path 输出的路径数组（2xMAX_PATH_POINTS）
 * @return 实际路径点数（包括结束标记）
 */
int planPath(int map[ROWS][COLS], int path[2][MAX_PATH_POINTS])
{
    // 初始化
    memcpy(g_map, map, sizeof(g_map));
    memset(g_visited, false, sizeof(g_visited));
    initPath(); // 路径引入
    g_path_index = 0;
    g_visited_count = 0;
    g_final_path_length = 0;

    // 统计可飞区域
    countFlyableAreas();
    // 从起点开始DFS
    if (dfs(6, 8))
    {
        // 添加结束标记
        if (g_final_path_length < MAX_PATH_POINTS)
        {
            g_final_path[0][g_final_path_length] = END_MARKER_ROW;
            g_final_path[1][g_final_path_length] = END_MARKER_COL;
            g_final_path_length++;
        }

        // 复制结果到输出数组
        for (int i = 0; i < g_final_path_length; i++)
        {
            path[0][i] = g_final_path[0][i];
            path[1][i] = g_final_path[1][i];
        }
        return g_final_path_length;
    }
    else
    {
        printf("Path planning failed: Cannot cover all flyable areas\n");
        return 0;
    }
}

/**
 * @brief 使用A*算法寻找从当前位置返回起点的最短路径
 * @param start_x 起始行坐标
 * @param start_y 起始列坐标
 * @param end_x 目标行坐标（起点）
 * @param end_y 目标列坐标（起点）
 * @param return_path 输出的返回路径
 * @param return_length 输出的返回路径长度
 * @return true-成功找到路径，false-失败
 */
bool findReturnPath(int start_x, int start_y, int end_x, int end_y,
                    int return_path[2][MAX_PATH_POINTS], int *return_length)
{
    // 使用BFS找最短路径
    bool visited[ROWS][COLS];
    int parent_x[ROWS][COLS];
    int parent_y[ROWS][COLS];
    memset(visited, false, sizeof(visited));
    memset(parent_x, -1, sizeof(parent_x));
    memset(parent_y, -1, sizeof(parent_y));

    // 队列
    int queue_x[ROWS * COLS];
    int queue_y[ROWS * COLS];
    int front = 0, rear = 0;

    // 起始点入队
    queue_x[rear] = start_x;
    queue_y[rear] = start_y;
    rear++;
    visited[start_x][start_y] = true;

    bool found = false;

    // BFS搜索
    while (front < rear && !found)
    {
        int cx = queue_x[front];
        int cy = queue_y[front];
        front++;

        // 检查是否到达目标
        if (cx == end_x && cy == end_y)
        {
            found = true;
            break;
        }

        // 探索四个方向
        for (int i = 0; i < 4; i++)
        {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (isValid(nx, ny) && !visited[nx][ny])
            {
                visited[nx][ny] = true;
                parent_x[nx][ny] = cx;
                parent_y[nx][ny] = cy;
                queue_x[rear] = nx;
                queue_y[rear] = ny;
                rear++;

                if (nx == end_x && ny == end_y)
                {
                    found = true;
                    break;
                }
            }
        }
    }

    if (!found)
    {
        return false;
    }

    // 回溯构建路径
    int temp_path_x[MAX_PATH_POINTS];
    int temp_path_y[MAX_PATH_POINTS];
    int path_count = 0;

    int cx = end_x;
    int cy = end_y;

    while (cx != start_x || cy != start_y)
    {
        temp_path_x[path_count] = cx;
        temp_path_y[path_count] = cy;
        path_count++;

        int px = parent_x[cx][cy];
        int py = parent_y[cx][cy];
        cx = px;
        cy = py;

        if (path_count >= MAX_PATH_POINTS - 1)
        {
            break;
        }
    }

    // 添加起始点
    temp_path_x[path_count] = start_x;
    temp_path_y[path_count] = start_y;
    path_count++;

    // 反转路径（因为是从终点回溯的）
    *return_length = 0;
    for (int i = path_count - 1; i >= 0; i--)
    {
        return_path[0][*return_length] = temp_path_x[i];
        return_path[1][*return_length] = temp_path_y[i];
        (*return_length)++;
    }

    return true;
}
int return_path[2][MAX_PATH_POINTS];
int return_length = 0;

extern char buffer2[200];
extern int len2;
/**
 * @brief 主路径规划函数（增强版：包含返回路径）
 * @param map 输入的地图数组（7x9）
 * @param path 输出的路径数组（2xMAX_PATH_POINTS）
 * @param include_return 是否包含返回起点的路径
 * @return 实际路径点数（包括结束标记）
 */
int forward_length = 0;
int planPathWithReturn(int map[ROWS][COLS], int path[2][MAX_PATH_POINTS], bool include_return)
{
    // 首先执行原有的路径规划
    forward_length = planPath(map, path);

    if (forward_length <= 0 || !include_return)
    {
        return forward_length;
    }

    // 获取最后一个有效位置（结束标记前）
    int last_x = path[0][forward_length - 2];
    int last_y = path[1][forward_length - 2];

    // 计算返回路径

    //  printf("\nCalculating return path...\n");
    if (findReturnPath(last_x, last_y, 6, 8, return_path, &return_length))
    {
        //    printf("Found return path, length: %d\n", return_length);

        // 将返回路径添加到主路径（跳过返回路径的第一个点，因为它与最后位置重复）
        int total_length = forward_length - 1; // 移除原来的结束标记

        for (int i = 1; i < return_length && total_length < MAX_PATH_POINTS - 1; i++)
        {
            path[0][total_length] = return_path[0][i];
            path[1][total_length] = return_path[1][i];
            total_length++;
        }

        // 添加新的结束标记
        path[0][total_length] = END_MARKER_ROW;
        path[1][total_length] = END_MARKER_COL;
        total_length++;

        return total_length;
    }
    else
    {
        printf("Warning: Cannot find return path!\n");
        return forward_length;
    }
}
char buffer[64]; // 确保缓冲区足够大
int len;
/**
 * @brief 输出包含返回路径的完整路径
 * @param path 路径数组
 * @param length 路径长度
 */
void outputPathWithReturn(int path[2][MAX_PATH_POINTS], int length)
{
    // 找出前向路径的结束位置
    int forward_end = -1;
    bool found_return = false;

    // 检测是否有返回到起点
    for (int i = length - 2; i > 0; i--)
    {
        if (path[0][i] == 6 && path[1][i] == 8)
        {
            forward_end = i;
            found_return = true;
            break;
        }
    }

    // 输出所有路径点
    //  printf("Complete path sequence:\n");
    for (int i = 0; i < length; i++)
    {
        if (path[0][i] == END_MARKER_ROW && path[1][i] == END_MARKER_COL)
        {
            printf("%d,%d\n", path[0][i], path[1][i]);
            //    printf("[%03d] (%d,%d) <-- End marker\n", i, path[0][i], path[1][i]);
        }
        else
        {

            // 标记特殊点
            if (i == 0)
            {
                printf(" <-- Start point");
            }
            else if (found_return && i == forward_end)
            {
                printf(" <-- Returned to start");
            }
            //         printf("\n");
        }
    }
    bool valid = true;
    int errors = 0;

    for (int i = 1; i < length - 1; i++)
    {
        int dx = abs(path[0][i] - path[0][i - 1]);
        int dy = abs(path[1][i] - path[1][i - 1]);

        if (dx + dy != 1)
        {
            errors++;
            valid = false;
        }
    }

    // printf("Path validation %s: ", valid ? "PASS" : "FAIL");
    if (!valid)
    {
        printf("Found %d illegal moves\n", errors);
    }
    else
    {
        printf("All moves are adjacent\n");
    }
    bool covered[ROWS][COLS];
    memset(covered, false, sizeof(covered));

    for (int i = 0; i < length - 1; i++)
    {
        int x = path[0][i];
        int y = path[1][i];
        if (x >= 0 && x < ROWS && y >= 0 && y < COLS)
        {
            covered[x][y] = true;
        }
    }
}

extern u8 anjian_temp;
extern u8 error_geshu;

extern u8 error_data[100];
extern char key_confirm;
extern u8 start_flag;
u8 end_flag = 0;
u8 send_i = 0;
#include "lcd_task.h"

extern ANIMALS Animal;
int time_count = 0;
extern u8 rx_buffer1[20];
u8 find_flag = 0;
u8 new_road = 0;

u8 openmv_receive[4];
u8 receive_last2[4];
extern u8 start2_flag;
int result_path[2][MAX_PATH_POINTS];
void Send_Task(void const *pvParameters)
{
    while (1)
    {
        if (start2_flag == 1)
        {
            int map[ROWS][COLS] = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1}};
            // 示例地图
            for (int i = 0; i < error_geshu; i++)
            {
                map[error_data[i] / 10][error_data[i] % 10] = 0;
            }
            // 初始化结果数组
            for (int i = 0; i < MAX_PATH_POINTS; i++)
            {
                result_path[0][i] = -1;
                result_path[1][i] = -1;
            }
            // printMap(map);

            // 选项1：只输出巡查路径
            int survey_only_length = planPath(map, result_path);

            // 重新初始化数组
            for (int i = 0; i < MAX_PATH_POINTS; i++)
            {
                result_path[0][i] = -1;
                result_path[1][i] = -1;
            }
            int complete_length = planPathWithReturn(map, result_path, true);
            if (complete_length > 0)
            {
                // 输出完整路径
                outputPathWithReturn(result_path, complete_length);
            }
            for (int i = 0; i < forward_length; i++)
            {
                len = sprintf(buffer, "line %d,%d,%d,%d,%d\xff\xff\xff", 260, 60, 800, 60, 0);
                HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
                if (i != 0)
                {
                    len = sprintf(buffer, "line %d,%d,%d,%d,%d\xff\xff\xff", 260 + 30 + 60 * result_path[1][i - 1], 60 + 30 + 60 * result_path[0][i - 1], 260 + 30 + 60 * result_path[1][i], 60 + 30 + 60 * result_path[0][i], 0);
                    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
                }
                osDelay(2);
            }

            for (int i = forward_length; i < complete_length - 1; i++)
            {
                len = sprintf(buffer, "line %d,%d,%d,%d,%d\xff\xff\xff", 270 + 30 + 60 * result_path[1][i - 1], 50 + 30 + 60 * result_path[0][i - 1], 270 + 30 + 60 * result_path[1][i], 50 + 30 + 60 * result_path[0][i], 31);
                HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
                osDelay(2);
            }
            // 1%c%c

						while(start_flag==0)  //延时
						{
							osDelay(1);
						}
            for (int i = 0; i < 30; i++)
            {
                printf("[9,9]"); // 前y后x
                osDelay(400);
            }
					//	osDelay(5000);

            for (send_i = 0; send_i < forward_length; send_i++)
            {
                while (time_count < 10) // 停留
                {
                    printf("[%d,%d]", result_path[0][send_i], result_path[1][send_i]); // 前y后x
                    //            printf("[%d,%d]",  result_path [0][send_i],result_path[1][send_i]);  //前y后x
                    osDelay(300);
                    time_count++;
                }
                new_road = 1;

                for (int i = 0; i <= send_i - 1; i++) //
                {
                    if (result_path[0][i] == result_path[0][send_i] && result_path[1][i] == result_path[1][send_i])
                    {
                        new_road = 0;
                    }
                }

                if (new_road == 1)
                {
                        for(int i = 0; i < 4; i++)
                        {
                            openmv_receive [i] = 0;
                            rx_buffer1[i] = 0;
                        }

                    for (int i = 0; i < 7; i++)  //一秒钟内有数据
                    {      printf("[%d,%d]", result_path[0][send_i], result_path[1][send_i]); // 前y后x

                        if (rx_buffer1[0]+rx_buffer1[1] + rx_buffer1[2] + rx_buffer1[3] + rx_buffer1[4] > 0) // 如果有数据
                        {
                            find_flag = 1;
                            break;
                        }
                        osDelay(200);
                    }

                    if (find_flag == 1)  //如果有数据 停留并取最大
                    {
                        for(int i = 0; i < 15; i++)
                        {
                        for(int i = 0; i < 4; i++)
                    {   if(rx_buffer1[i] >openmv_receive[i])
                        {
                            openmv_receive [i] = 1;//rx_buffer1[i];
                        }
                  
                    }
										printf("[%d,%d]", result_path[0][send_i], result_path[1][send_i]); // 前y后x
                    osDelay(200);

                }
                
                for(int i = 0; i < 4; i++)
                {
                    for(int j = 0; j < openmv_receive[i]; j++)
                    {
                        Animal.animals[i].place[Animal.animals[i].count]=result_path[0][send_i]*10+result_path[1][send_i];
                        Animal.animals[i].count++;
                    }
                }
								
								for (int ele = 0; ele < Animal.animals[0].count; ele++) // 大象
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   ele * 40, 50, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[0].place[ele]/10), Animal.animals[0].place[ele]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int tig = 0; tig < Animal.animals[1].count; tig++) // 老虎
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   tig * 40, 150, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[1].place[tig]/10), Animal.animals[1].place[tig]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int wol = 0; wol < Animal.animals[2].count; wol++) // 狼
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   wol * 40, 250, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[2].place[wol]/10), Animal.animals[2].place[wol]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int mon = 0; mon < Animal.animals[3].count; mon++) // 猴子
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   mon * 40, 350, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[3].place[mon]/10), Animal.animals[3].place[mon]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}
								
								
								
								  find_flag = 0;
            }
                        find_flag = 0;
                time_count = 0;
            }
								                        for(int i = 0; i < 4; i++)
                        {
                            openmv_receive[i] = 0;
                            rx_buffer1[i] = 0;
                        }
        }

            for (send_i = forward_length; send_i < complete_length; send_i++)  //结束
            {

                while (time_count < 7) // 停留
                {
                    printf("[%d,%d]", result_path[0][send_i], result_path[1][send_i]); // 前y后x
                    //            printf("[%d,%d]",  result_path [0][send_i],result_path[1][send_i]);  //前y后x
                    osDelay(300);
                    time_count++;
                }

                time_count = 0;
            }
            send_i -= 1;
            start_flag = 0;
            end_flag = 10;
						start2_flag=0;
        }
        else
        {
            osDelay(100);
        }
            }}
        

