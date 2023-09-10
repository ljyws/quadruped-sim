#include "dynacore_yaml-cpp/node/emit.h"
#include "dynacore_yaml-cpp/emitfromevents.h"
#include "dynacore_yaml-cpp/emitter.h"
#include "nodeevents.h"

namespace dynacore_YAML {
Emitter& operator<<(Emitter& out, const Node& node) {
  EmitFromEvents emitFromEvents(out);
  NodeEvents events(node);
  events.Emit(emitFromEvents);
  return out;
}

std::ostream& operator<<(std::ostream& out, const Node& node) {
  Emitter emitter(out);
  emitter << node;
  return out;
}

std::string Dump(const Node& node) {
  Emitter emitter;
  emitter << node;
  return emitter.c_str();
}
}  // namespace YAML
