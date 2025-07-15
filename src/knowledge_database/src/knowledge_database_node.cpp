#include "rclcpp/rclcpp.hpp"
#include "collaborative_intelligence/msg/knowledge.hpp"
#include <sqlite3.h>

class KnowledgeDatabase : public rclcpp::Node
{
public:
  KnowledgeDatabase()
  : Node("knowledge_database")
  {
    int rc = sqlite3_open("knowledge.db", &db_);
    if (rc) {
      RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Opened database successfully");
    }

    char *zErrMsg = 0;
    const char *sql = "CREATE TABLE KNOWLEDGE(" \
                      "ID INT PRIMARY KEY     NOT NULL," \
                      "TYPE           TEXT    NOT NULL," \
                      "DATA           TEXT    NOT NULL);";
    rc = sqlite3_exec(db_, sql, 0, 0, &zErrMsg);
    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(this->get_logger(), "SQL error: %s", zErrMsg);
      sqlite3_free(zErrMsg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Table created successfully");
    }

    subscription_ = this->create_subscription<collaborative_intelligence::msg::Knowledge>(
      "knowledge", 10, std::bind(&KnowledgeDatabase::topic_callback, this, std::placeholders::_1));
  }

  ~KnowledgeDatabase()
  {
    sqlite3_close(db_);
  }

private:
  void topic_callback(const collaborative_intelligence::msg::Knowledge::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received knowledge of type %s", msg->type.c_str());

    char *zErrMsg = 0;
    std::string sql = "INSERT INTO KNOWLEDGE (ID,TYPE,DATA) VALUES (NULL, '" + msg->type + "', '" + msg->data + "');";
    int rc = sqlite3_exec(db_, sql.c_str(), 0, 0, &zErrMsg);
    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(this->get_logger(), "SQL error: %s", zErrMsg);
      sqlite3_free(zErrMsg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Record created successfully");
    }
  }

  sqlite3 *db_;
  rclcpp::Subscription<collaborative_intelligence::msg::Knowledge>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KnowledgeDatabase>());
  rclcpp::shutdown();
  return 0;
}
