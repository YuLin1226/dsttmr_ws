# Development

1. 需要兩個 `move_base`
2. 需要客製化的 `Global Planner`，並且請善用 `.msg`
3. 可能會需要先前寫到的 `cpp library`，像是 `tf_listener`
4. 中心點的導航路徑直接外包成一個 `node`，並且採用 `Bezier Curve` 來操作軌跡。