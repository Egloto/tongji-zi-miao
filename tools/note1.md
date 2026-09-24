auto yaml = tools::load(config_path);
> 调用tool里面yamel这个文件
>
>问题： 为什么直接调取hpp文件就实现了路径的配置？

因为有inline函数 inline 告诉编译器"把这个函数的代码直接嵌入调用处，不要单独生成函数"。对于这种只有两行的小包装函数，写在 .hpp 里省得多建一个 .cpp 文件。
```cpp
namespace tools
{
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP
```
>