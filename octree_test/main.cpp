// Copyright (c) <year> Your name

#include "engine/easy.h"
#include "engine/bitstream.h"

#include <nmmintrin.h>

using namespace arctic;  // NOLINT
using namespace arctic::easy;  // NOLINT

Si32 g_ser = 0;

double g_prev_time;
double g_cur_time;
Font g_font;

double g_displayed_dt = 1.0/60.0;
double g_time_to_displayed_dt_change = 0.0;

// Tight octree bitstream format:
// Nodes start from root and then in depth-first order
// arrays of child bit-records for each node follow:
// 0 - void leaf
// 1 - non-void entry
//     0 - mixed node
//     1 - solid leaf

// node_0: flags for child nodes [01001011]
// 
// child offset: node_1_000
//               node_1_001
//               node_1_011
//               node_1_110

struct OctreeNode {
  Ui32 child_offset = 0; // 0 == leaf node, other offet indicates first non-empty child position
  Ui8 child_mask = 0; // 1 for n-th bit indicates that the n-th child is present
  Ui8 r = 0;
  Ui8 g = 0;
  Ui8 b = 0;
  Ui8 n_x = 0;
  Ui8 n_y = 0;
  Ui8 flags = 0; // bit 0 LOD_X: 0 for transparent LOD_X, 1 for opaque
                 // bit 1 LOD_Y: 0 for transparent LOD_Y, 1 for opaque
                 // bit 2 LOD_Z: 0 for transparent LOD_Z, 1 for opaque
  Ui8 reserved = 0;
};


struct FatOctreeNode {
  enum Solidity {
    kUnknown = 0,
    kVoid = 1,
    kMixed = 2,
    kSolid = 3
  };
  Ui32 child_offset;
  float r;
  float g;
  float b;
  float a; // 0 == fully transparent, 1 == fully opaque
  FatOctreeNode* children[8];
  bool is_serialized;
  Ui8 child_mask; // 1 for n-th bit indicates that the n-th child is present
  Solidity solidity;

  void FreeChildren();
  void AddChild(Ui32 i, FatOctreeNode *c);
  void ProcessChildren();
};

union FatOctreeNodePoolItem {
  FatOctreeNodePoolItem *next;
  FatOctreeNode data;
};

std::vector<FatOctreeNodePoolItem> g_fat_node_pool_data;
FatOctreeNodePoolItem *g_fat_node_pool_head = nullptr;

void InitFatNodePool(Ui32 size) {
  g_fat_node_pool_data.resize(size);
  g_fat_node_pool_head = &g_fat_node_pool_data[0];
  for (size_t i = 1; i < size; ++i) {
    g_fat_node_pool_data[i-1].next = &g_fat_node_pool_data[i];
  }
  g_fat_node_pool_data[size - 1].next = nullptr;
}

FatOctreeNode* AllocateFatNode() {
  Check(g_fat_node_pool_head, "Fat node pool exhausted!"); 
  FatOctreeNodePoolItem* item = g_fat_node_pool_head;
  g_fat_node_pool_head = g_fat_node_pool_head->next;
  item->next = nullptr;
  return &(item->data);
}

void FreeFatNode(FatOctreeNode *node) {
  FatOctreeNodePoolItem *p = reinterpret_cast<FatOctreeNodePoolItem*>(node);
  Check(p != nullptr, "cast error");
  p->next = g_fat_node_pool_head;
  g_fat_node_pool_head = p;
}

void FatOctreeNodeToOctreeNode(FatOctreeNode &n, OctreeNode *s) { 
  s->child_offset = n.child_offset;
  s->child_mask = n.child_mask;
  s->r = n.r * 255.f;
  s->g = n.g * 255.f;
  s->b = n.b * 255.f;
  s->flags = 7;
}

void OutputOctreeNode(FatOctreeNode *n, std::deque<OctreeNode> *out_octree) { 
  for (Ui32 i = 0; i < 8; ++i) {
    FatOctreeNode *c = n->children[i];
    if (c && c->solidity != FatOctreeNode::kVoid) {
      n->child_mask |= (1 << i);
      if (n->child_offset == 0) {
        n->child_offset = (Ui32)out_octree->size();
      }
      out_octree->emplace_back();
      FatOctreeNodeToOctreeNode(*c, &out_octree->back());
    }
  }
}

FatOctreeNode* FillNode(const Vec3Si32 pos, const Ui32 side,
    const Ui32 raw_side, const std::vector<Ui8> &raw, std::deque<OctreeNode> *out_octree) {
  //*(Log()) << "FillNode (" << pos.x << ", " << pos.y << ", " << pos.z << ") " << side;
  FatOctreeNode *n = AllocateFatNode();
  memset(n, 0, sizeof(FatOctreeNode));
  if (side == 1) {
    const Ui8 a = raw[pos.x + pos.y * raw_side + pos.z * raw_side * raw_side];
    n->a = a / 255.f;
    n->solidity = a ? FatOctreeNode::kSolid : FatOctreeNode::kVoid;
    return n;
  }
  const Ui32 child_side = (side >> 1);
  for (Ui32 i = 0; i < 8; ++i) {
    const Ui32 xo = i & 1;
    const Ui32 yo = (i >> 1) & 1;
    const Ui32 zo = (i >> 2) & 1;
    const Vec3Si32 child_pos(pos.x + xo * child_side, pos.y + yo * child_side, pos.z + zo * child_side);
    FatOctreeNode *c = FillNode(child_pos, child_side, raw_side, raw, out_octree);
    n->AddChild(i, c);
  }
  n->ProcessChildren();
  if (n->solidity == FatOctreeNode::kMixed) {
    OutputOctreeNode(n, out_octree); 
  }
  n->FreeChildren();
  return n;
}


void FatOctreeNode::FreeChildren() {
  for (Ui32 i = 0; i < 8; ++i) {
    if (children[i]) {
      FreeFatNode(children[i]);
      children[i] = nullptr;
    }
  }
}
void FatOctreeNode::AddChild(Ui32 i, FatOctreeNode *c) {
  children[i] = c;
  r += c->r * c->a;
  g += c->g * c->a;
  b += c->b * c->a;
  a += c->a;
}
void FatOctreeNode::ProcessChildren() {
  if (a > 0.f) {
    r /= a;
    g /= a;
    b /= a;
  }
  a /= 8.f;
  if (a == 1.f) {
    solidity = FatOctreeNode::kSolid;
    for (Ui32 i = 0; i < 7; ++i) {
      FatOctreeNode *c1 = children[i];
      FatOctreeNode *c2 = children[i + 1];
      if (!c1 || !c2 || c1->r != c2->r || c1->g != c2->g || c1->b != c2->b || c1->a != c2->a ||
          c1->solidity != FatOctreeNode::kSolid || c2->solidity != FatOctreeNode::kSolid) {
        solidity = FatOctreeNode::kMixed;
        break;
      }
    }
  } else if (a == 0.f) {
    solidity = FatOctreeNode::kVoid;
  } else {
    solidity = FatOctreeNode::kMixed;
  }
}

void RawToOctree(const std::vector<Ui8> &raw, std::vector<OctreeNode> *out_octree) {
  std::deque<OctreeNode> octree;
  octree.emplace_back();
  FatOctreeNode* c = FillNode(Vec3Si32(0, 0, 0), 512, 512, raw, &octree);
  FatOctreeNodeToOctreeNode(*c, &octree.front()); 
  out_octree->resize(octree.size());
  std::copy(octree.begin(), octree.end(), out_octree->begin());
}

// each node record contains a solidity flag
// 0 - mixed node
// 1 - solid leaf
// mixed nodes contain 8 bits of the child_mask
// 0 - void leaf
// 1 - non-void entry
// then child node records follow for non-void entries

void OctreeToTightOctree(std::vector<OctreeNode> &octree, Si32 node_offset,
    BitStream *out_tight_octree) {
  OctreeNode &cur = octree[node_offset];
  out_tight_octree->PushBit(cur.child_offset ? 0 : 1);
  if (cur.child_offset) {
    for (Si32 i = 0; i < 8; ++i) {
      out_tight_octree->PushBit((cur.child_mask >> i) & 1);
    }
    for (Ui32 i = 0; i < 8; ++i) {
      if ((cur.child_mask >> i) & 1) {
        Ui32 idx = __builtin_popcount(Ui32(cur.child_mask) & (Ui32(0xff) >> (8 - i)));
        OctreeToTightOctree(octree, cur.child_offset + idx, out_tight_octree);
      }
    }
  }
}

FatOctreeNode* FillNodeFromTight(BitStream &tight_octree, std::deque<OctreeNode> *out_octree) {
  FatOctreeNode *n = AllocateFatNode();
  memset(n, 0, sizeof(FatOctreeNode));
  Ui8 is_leaf = (tight_octree.ReadBit() & 1);
  if (is_leaf) {
    n->a = 1.f;
    n->solidity = FatOctreeNode::kSolid;
    return n;
  }

  Ui8 child_mask = 0;
  for (Ui32 i = 0; i < 8; ++i) {
    child_mask |= ((tight_octree.ReadBit() & 1) << i);
  }
  Check(child_mask, "Unexpected empty child mask");
  for (Ui32 i = 0; i < 8; ++i) {
    FatOctreeNode *c = nullptr;
    if ((child_mask >> i) & 1) {
      c = FillNodeFromTight(tight_octree, out_octree);
    } else {
      c = AllocateFatNode();
      memset(c, 0, sizeof(FatOctreeNode));
      c->a = 0.f;
      c->solidity = FatOctreeNode::kVoid;
    }
    n->AddChild(i, c);
  }
  n->ProcessChildren();
  if (n->solidity == FatOctreeNode::kMixed) {
    OutputOctreeNode(n, out_octree); 
  }
  n->FreeChildren();
  return n;
}

void TightOctreeToOctree(BitStream &tight_octree, std::vector<OctreeNode> *out_octree) {
  std::deque<OctreeNode> octree;
  octree.emplace_back();
  FatOctreeNode* c = FillNodeFromTight(tight_octree, &octree);
  FatOctreeNodeToOctreeNode(*c, &octree.front()); 
  out_octree->resize(octree.size());
  std::copy(octree.begin(), octree.end(), out_octree->begin());
}


void EasyMain() {
  InitFatNodePool(1024);
  std::vector<Ui8> raw = ReadFile("data/bunny_512.raw");
  Check(raw.size() == 512 * 512 * 512, "Unexpected raw size");
  std::vector<OctreeNode> octree;
  RawToOctree(raw, &octree);
  WriteFile("data/bunny_512.svo", reinterpret_cast<const Ui8*>(octree.data()), sizeof(OctreeNode)*octree.size());

  BitStream tight_octree;
  OctreeToTightOctree(octree, 0, &tight_octree);
  std::vector<Ui8> tight_vec(tight_octree.GetData().size());
  std::copy(tight_octree.GetData().begin(), tight_octree.GetData().end(), tight_vec.begin());
  WriteFile("data/bunny_512.tig", reinterpret_cast<const Ui8*>(tight_vec.data()), tight_vec.size());

  std::vector<Ui8> tight_vec_2 = ReadFile("data/bunny_512.tig");
  BitStream tight_octree_2(tight_vec_2);
  std::vector<OctreeNode> octree_2;
  TightOctreeToOctree(tight_octree_2, &octree_2);
  WriteFile("data/bunny_512_2.svo", reinterpret_cast<const Ui8*>(octree_2.data()), sizeof(OctreeNode)*octree_2.size());


  ResizeScreen(1920, 1080);
  g_font.Load("data/arctic_one_bmf.fnt");
  g_cur_time = Time();
  while (!IsKeyDownward(kKeyEscape)) {
    g_prev_time = g_cur_time;
    g_cur_time = Time();
    double dt = g_cur_time - g_prev_time;
    g_time_to_displayed_dt_change -= dt;
    if (g_time_to_displayed_dt_change <= 0.0) {
      g_time_to_displayed_dt_change = 0.2;
      g_displayed_dt = dt;
    }

    Clear();

    char message[4096];
    snprintf(message, 4096, "FPS: %2.1f (%2.2f msPF) octree: %zu nodes (%zu) %u",
        1.f / std::max(1e-9, g_displayed_dt),
        g_displayed_dt*1000.f,
        octree.size(),
        octree_2.size(),
        g_ser
        );
    g_font.Draw(message, 0, ScreenSize().y, kTextOriginTop);
    ShowFrame();
  }
}
