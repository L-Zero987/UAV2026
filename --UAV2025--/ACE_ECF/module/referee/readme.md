使用：

创建画ui的对象，设定串口：

REFEREE_n::CILENT_UI_n::UI_Draw_c ui_draw(&huart6);

设置机器人ID（这部分需要改一下现在只有步兵的）

  ui_draw.Set_robo_id(Referee_Get_robo_id());

  /定义一个直线

  auto line1 = std::make_unique<REFEREE_n::CILENT_UI_n::LineCommand>("143", 600, 100, 800, 500);

设置为添加操作（默认）

line1 ->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_ADD);

添加后才可以进行修改：

line1 ->configure(REFEREE_n::CILENT_UI_n::Graph_Operate_e::UI_Graph_Change);

画出来：

ui_draw.Draw_A_Graph(line1 .get());







画一组的操作：

std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc1 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>("123", X_MAX/2, Y_MAX/2, 0, 0,50,50);

  std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc2 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>("124", X_MAX/2, Y_MAX/2, 0, 0,100,100);

  std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc3 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>("125", X_MAX/2, Y_MAX/2, 0, 0,150,150);

  std::unique_ptr<REFEREE_n::CILENT_UI_n::ArcCommand> arc4 = std::make_unique<REFEREE_n::CILENT_UI_n::ArcCommand>("126", X_MAX/2, Y_MAX/2, 0, 0,200,200);

 

  // 定义vector数组

  std::vector<REFEREE_n::CILENT_UI_n::GraphicCommand*> graphic_group;

  graphic_group.push_back(arc1.get());

  graphic_group.push_back(arc2.get());

  graphic_group.push_back(arc3.get());

  graphic_group.push_back(arc4.get());

画一组：

ui_draw.Draw_Group_Graph(graphic_group);