import torch
import torch.nn as nn
import torch.nn.functional as F
import timm
from torchvision.ops import roi_align


# ── Backbone ──────────────────────────────────────────────────────────────
class Backbone(nn.Module):
    SUPPORTED = [
        'convnext_small',
        'resnet50',
        'efficientnet_b3',
        'mobilenetv3_large_100',
        'regnety_008',
    ]

    def __init__(self, name: str = 'convnext_small', pretrained: bool = True):
        super().__init__()
        assert name in self.SUPPORTED, f'Choose from {self.SUPPORTED}'
        self.net = timm.create_model(name, features_only=True,
                                     pretrained=pretrained)
        all_channels = self.net.feature_info.channels()
        all_strides  = self.net.feature_info.reduction()
        self.level_idx   = sorted([i for i, s in enumerate(all_strides)
                                   if s in (8, 16, 32)])
        self.out_channels = [all_channels[i] for i in self.level_idx]

    def forward(self, x):
        all_feats = self.net(x)
        return [all_feats[i] for i in self.level_idx]


# ── BiFPN ─────────────────────────────────────────────────────────────────
class DepthwiseSepConv(nn.Module):
    def __init__(self, ch):
        super().__init__()
        self.dw = nn.Conv2d(ch, ch, 3, padding=1, groups=ch, bias=False)
        self.pw = nn.Conv2d(ch, ch, 1, bias=False)
        self.bn = nn.BatchNorm2d(ch)

    def forward(self, x):
        return F.relu(self.bn(self.pw(self.dw(x))), inplace=True)


class BiFPNLayer(nn.Module):
    def __init__(self, ch: int, num_levels: int = 3, eps: float = 1e-4):
        super().__init__()
        self.eps        = eps
        self.num_levels = num_levels
        self.td_weights = nn.ParameterList(
            [nn.Parameter(torch.ones(2)) for _ in range(num_levels - 1)])
        self.bu_weights = nn.ParameterList(
            [nn.Parameter(torch.ones(3)) for _ in range(num_levels - 1)])
        self.td_convs   = nn.ModuleList(
            [DepthwiseSepConv(ch) for _ in range(num_levels - 1)])
        self.bu_convs   = nn.ModuleList(
            [DepthwiseSepConv(ch) for _ in range(num_levels - 1)])
        self.downsample = nn.MaxPool2d(2, 2)

    def _fuse2(self, w, x1, x2):
        w1, w2 = F.relu(w[0]), F.relu(w[1])
        return (w1 * x1 + w2 * x2) / (w1 + w2 + self.eps)

    def _fuse3(self, w, x1, x2, x3):
        w1, w2, w3 = F.relu(w[0]), F.relu(w[1]), F.relu(w[2])
        return (w1 * x1 + w2 * x2 + w3 * x3) / (w1 + w2 + w3 + self.eps)

    def forward(self, features):
        P = list(features)
        N = self.num_levels
        td     = [None] * N
        td[-1] = P[-1]
        for i in range(N - 2, -1, -1):
            up    = F.interpolate(td[i+1], size=P[i].shape[-2:], mode='nearest')
            td[i] = self.td_convs[i](self._fuse2(self.td_weights[i], P[i], up))
        out    = [None] * N
        out[0] = td[0]
        for i in range(1, N):
            down = self.downsample(out[i-1])
            if down.shape[-2:] != td[i].shape[-2:]:
                down = F.interpolate(down, size=td[i].shape[-2:], mode='nearest')
            out[i] = self.bu_convs[i-1](
                self._fuse3(self.bu_weights[i-1], P[i], td[i], down))
        return out


class BiFPN(nn.Module):
    def __init__(self, in_channels: list, out_ch: int = 256, num_layers: int = 3):
        super().__init__()
        self.input_proj = nn.ModuleList([
            nn.Sequential(
                nn.Conv2d(c, out_ch, 1, bias=False),
                nn.BatchNorm2d(out_ch),
                nn.ReLU(inplace=True),
            ) for c in in_channels
        ])
        self.layers = nn.ModuleList([
            BiFPNLayer(out_ch, num_levels=len(in_channels))
            for _ in range(num_layers)
        ])

    def forward(self, features):
        x = [proj(f) for proj, f in zip(self.input_proj, features)]
        for layer in self.layers:
            x = layer(x)
        return x


# ── CBAM ──────────────────────────────────────────────────────────────────
class ChannelAttention(nn.Module):
    def __init__(self, ch: int, reduction: int = 16):
        super().__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.max_pool = nn.AdaptiveMaxPool2d(1)
        self.mlp = nn.Sequential(
            nn.Conv2d(ch, ch // reduction, 1, bias=False),
            nn.ReLU(inplace=True),
            nn.Conv2d(ch // reduction, ch, 1, bias=False),
        )

    def forward(self, x):
        return torch.sigmoid(self.mlp(self.avg_pool(x)) +
                             self.mlp(self.max_pool(x)))


class SpatialAttention(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Conv2d(2, 1, 7, padding=3, bias=False)

    def forward(self, x):
        avg = x.mean(dim=1, keepdim=True)
        mx  = x.max(dim=1, keepdim=True).values
        return torch.sigmoid(self.conv(torch.cat([avg, mx], dim=1)))


class CBAM(nn.Module):
    def __init__(self, ch: int, reduction: int = 16):
        super().__init__()
        self.ca = ChannelAttention(ch, reduction)
        self.sa = SpatialAttention()

    def forward(self, x):
        x = x * self.ca(x)
        x = x * self.sa(x)
        return x


# ── Detection Head (obj + box only, no cls) ───────────────────────────────
class DetectionHead(nn.Module):
    def __init__(self, in_ch: int = 256, num_convs: int = 4):
        super().__init__()
        tower = []
        for _ in range(num_convs):
            tower += [
                nn.Conv2d(in_ch, in_ch, 3, padding=1, bias=False),
                nn.GroupNorm(32, in_ch),
                nn.ReLU(inplace=True),
            ]
        tower += [nn.Dropout2d(p=0.1)]
        self.tower    = nn.Sequential(*tower)
        self.box_head = nn.Conv2d(in_ch, 4, 1)
        self.obj_head = nn.Conv2d(in_ch, 1, 1)
        nn.init.normal_(self.box_head.weight, std=0.01)
        nn.init.zeros_(self.box_head.bias)

    def forward_single(self, feat, stride, img_h, img_w):
        B, _, H, W = feat.shape
        x = self.tower(feat)
        obj_logits = self.obj_head(x)
        box_raw    = self.box_head(x)

        gy = torch.arange(H, device=feat.device, dtype=torch.float32)
        gx = torch.arange(W, device=feat.device, dtype=torch.float32)
        grid_y, grid_x = torch.meshgrid(gy, gx, indexing='ij')
        cx_base = (grid_x + 0.5) / W
        cy_base = (grid_y + 0.5) / H
        cx = cx_base + torch.tanh(box_raw[:, 0]) * (1.0 / W)
        cy = cy_base + torch.tanh(box_raw[:, 1]) * (1.0 / H)
        w  = torch.clamp(box_raw[:, 2].exp() * stride / img_w, 0.01, 1.0)
        h  = torch.clamp(box_raw[:, 3].exp() * stride / img_h, 0.01, 1.0)
        box_preds = torch.stack([cx, cy, w, h], dim=1)
        return box_preds, obj_logits

    def forward(self, features, img_h, img_w):
        strides = [8, 16, 32]
        return [self.forward_single(f, s, img_h, img_w)
                for f, s in zip(features, strides)]


# ── Mask Decoder ──────────────────────────────────────────────────────────
class MaskDecoder(nn.Module):
    def __init__(self, in_ch: int = 256, num_classes: int = 5):
        super().__init__()
        self.dec2 = self._block(in_ch + in_ch, in_ch // 2)
        self.dec1 = self._block(in_ch // 2 + in_ch, in_ch // 4)
        self.dec0 = self._block(in_ch // 4, in_ch // 8)
        self.out  = nn.Conv2d(in_ch // 8, num_classes, 1)

    @staticmethod
    def _block(in_c, out_c):
        return nn.Sequential(
            nn.Conv2d(in_c, out_c, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_c), nn.ReLU(inplace=True),
            nn.Conv2d(out_c, out_c, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_c), nn.ReLU(inplace=True),
        )

    def forward(self, att_features: list, img_h: int, img_w: int):
        f3, f4, f5 = att_features
        d2 = self.dec2(torch.cat([
            F.interpolate(f5, size=f4.shape[-2:],
                          mode='bilinear', align_corners=False), f4
        ], dim=1))
        d1 = self.dec1(torch.cat([
            F.interpolate(d2, size=f3.shape[-2:],
                          mode='bilinear', align_corners=False), f3
        ], dim=1))
        d0 = self.dec0(d1)
        return F.interpolate(self.out(d0), size=(img_h, img_w),
                             mode='bilinear', align_corners=False)


# ── Class Head ────────────────────────────────────────────────────────────
class ClassHead(nn.Module):
    def __init__(self, in_ch: int = 256, num_classes: int = 5, roi_size: int = 7):
        super().__init__()
        self.roi_size    = roi_size
        self.num_classes = num_classes
        self.cbam        = CBAM(in_ch)
        flat = in_ch * roi_size * roi_size
        self.fc = nn.Sequential(
            nn.Linear(flat, 1024),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(1024, num_classes),
        )

    @staticmethod
    def _assign_level(boxes_norm, img_h, img_w):
        w_px = boxes_norm[:, 2] * img_w
        h_px = boxes_norm[:, 3] * img_h
        s    = (w_px * h_px + 1e-6).sqrt()
        return (s / 56.0 + 1e-6).log2().floor().long().clamp(0, 2)

    def forward(self, att_features, boxes_norm, batch_idx, img_h, img_w):
        if boxes_norm.shape[0] == 0:
            return torch.zeros(0, self.num_classes,
                               device=att_features[0].device)
        lvl_ids = self._assign_level(boxes_norm, img_h, img_w)
        out     = torch.zeros(boxes_norm.shape[0], self.num_classes,
                              device=att_features[0].device)
        for lvl_idx, feat in enumerate(att_features):
            mask = (lvl_ids == lvl_idx)
            if mask.sum() == 0:
                continue
            _, _, fH, fW = feat.shape
            b  = boxes_norm[mask]
            bi = batch_idx[mask]
            cx, cy, w, h = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
            x1 = (cx - w/2) * fW;  y1 = (cy - h/2) * fH
            x2 = (cx + w/2) * fW;  y2 = (cy + h/2) * fH
            rois  = torch.stack([bi.float(), x1, y1, x2, y2], dim=1)
            crops = roi_align(feat, rois, output_size=self.roi_size,
                              spatial_scale=1.0, aligned=True)
            crops     = self.cbam(crops)
            out[mask] = self.fc(crops.flatten(1)).float()
        return out


# ── Segmenta ──────────────────────────────────────────────────────────────
class Segmenta(nn.Module):
    def __init__(self, cfg: dict):
        super().__init__()
        self.cfg = cfg
        nc  = cfg['num_classes']
        ch  = cfg.get('fpn_ch', 256)
        nly = cfg.get('fpn_layers', 3)

        self.backbone     = Backbone(cfg['backbone'])
        self.bifpn        = BiFPN(self.backbone.out_channels,
                                  out_ch=ch, num_layers=nly)
        self.cbam_levels  = nn.ModuleList([CBAM(ch) for _ in range(3)])
        self.det_head     = DetectionHead(in_ch=ch)
        self.mask_decoder = MaskDecoder(in_ch=ch, num_classes=nc)
        self.class_head   = ClassHead(in_ch=ch, num_classes=nc)

    def forward(self, x):
        backbone_feats = self.backbone(x)
        fpn_features   = self.bifpn(backbone_feats)
        att_features   = [cbam(f) for cbam, f
                          in zip(self.cbam_levels, fpn_features)]
        img_h, img_w   = x.shape[-2], x.shape[-1]
        det_outputs    = self.det_head(att_features, img_h, img_w)
        seg_map        = self.mask_decoder(att_features, img_h, img_w)
        return det_outputs, att_features, seg_map

    @torch.no_grad()
    def predict(self, x):
        self.eval()
        img_h, img_w  = x.shape[-2], x.shape[-1]
        B             = x.shape[0]
        det_outputs, att_features, seg_map = self.forward(x)

        seg_prob    = seg_map.softmax(dim=1)
        fg_prob     = 1.0 - seg_prob[:, 0]
        obj_thresh  = self.cfg.get('score_thresh', 0.5)
        mask_thresh = self.cfg.get('mask_thresh',  0.4)

        results = []
        for b in range(B):
            boxes_list, scores_list = [], []
            for box_preds, obj_logits in det_outputs:
                obj_prob = obj_logits[b, 0].sigmoid()
                pos = obj_prob > obj_thresh
                if pos.sum() == 0:
                    continue
                scores_list.append(obj_prob[pos])
                boxes_list.append(box_preds[b].permute(1, 2, 0)[pos])
            if not boxes_list:
                results.append([])
                continue
            boxes  = torch.cat(boxes_list)
            scores = torch.cat(scores_list)

            # Step 1 — Global fg vote
            fg = fg_prob[b]
            keep1 = []
            for i in range(boxes.shape[0]):
                cx, cy, w, h = boxes[i].tolist()
                x1 = int(max((cx - w / 2) * img_w, 0))
                y1 = int(max((cy - h / 2) * img_h, 0))
                x2 = int(min((cx + w / 2) * img_w, img_w))
                y2 = int(min((cy + h / 2) * img_h, img_h))
                region = fg[y1:y2, x1:x2]
                if region.numel() > 0 and \
                        (region > 0.5).float().mean().item() >= 0.20:
                    keep1.append(i)
            if not keep1:
                results.append([])
                continue
            boxes  = boxes[keep1]
            scores = scores[keep1]

            # Class head
            single_att  = [f[b:b+1] for f in att_features]
            batch_zeros = torch.zeros(boxes.shape[0], dtype=torch.long,
                                      device=x.device)
            cls_logits  = self.class_head(
                single_att, boxes, batch_zeros, img_h, img_w)
            cls_probs   = cls_logits.softmax(dim=1)
            cls_ids     = cls_probs[:, 1:].argmax(dim=1) + 1
            cls_scores  = cls_probs[:, 1:].max(dim=1).values

            # Step 2 — Crop class channel as instance mask
            full_masks, refined, keep2_boxes, keep2_scores = [], [], [], []
            keep2_cls_ids, keep2_cls_scores = [], []

            for i in range(boxes.shape[0]):
                cls_id   = cls_ids[i].item()
                cx, cy, w, h = boxes[i].tolist()

                ew = min(w * 1.5, 1.0)
                eh = min(h * 1.5, 1.0)
                ex1 = int(max((cx - ew / 2) * img_w, 0))
                ey1 = int(max((cy - eh / 2) * img_h, 0))
                ex2 = int(min((cx + ew / 2) * img_w, img_w))
                ey2 = int(min((cy + eh / 2) * img_h, img_h))

                cls_channel = seg_prob[b, cls_id]
                canvas = torch.zeros(img_h, img_w, dtype=torch.bool,
                                     device=x.device)
                canvas[ey1:ey2, ex1:ex2] = \
                    cls_channel[ey1:ey2, ex1:ex2] > mask_thresh

                if canvas.sum() == 0:
                    continue

                bw = max(ex2 - ex1, 1)
                bh = max(ey2 - ey1, 1)
                if canvas.sum().item() / (bw * bh) < 0.15:
                    continue

                full_masks.append(canvas)

                ys, xs = canvas.nonzero(as_tuple=True)
                rx1, ry1 = xs.min().item(), ys.min().item()
                rx2, ry2 = xs.max().item() + 1, ys.max().item() + 1
                refined.append(torch.tensor([
                    (rx1 + rx2) / 2 / img_w, (ry1 + ry2) / 2 / img_h,
                    (rx2 - rx1) / img_w,      (ry2 - ry1) / img_h,
                ], device=x.device))
                keep2_boxes.append(boxes[i])
                keep2_scores.append(scores[i])
                keep2_cls_ids.append(cls_ids[i])
                keep2_cls_scores.append(cls_scores[i])

            if not refined:
                results.append([])
                continue

            boxes      = torch.stack(refined)
            scores     = torch.stack(keep2_scores)
            cls_ids    = torch.stack(keep2_cls_ids)
            cls_scores = torch.stack(keep2_cls_scores)

            # Step 3 — Mask pixel NMS
            N          = boxes.shape[0]
            suppressed = torch.zeros(N, dtype=torch.bool, device=x.device)
            order      = scores.argsort(descending=True)
            keep5      = []
            for ii in range(N):
                i = order[ii].item()
                if suppressed[i]:
                    continue
                keep5.append(i)
                mi = full_masks[i].float()
                for jj in range(ii + 1, N):
                    j = order[jj].item()
                    if suppressed[j]:
                        continue
                    if (mi * full_masks[j].float()).sum() / \
                            mi.sum().clamp(1) > 0.5:
                        suppressed[j] = True
            if not keep5:
                results.append([])
                continue

            # Pack results
            instances = []
            for i in keep5:
                cx, cy, w, h = boxes[i].tolist()
                instances.append({
                    'class'     : cls_ids[i].item(),
                    'obj_score' : scores[i].item(),
                    'cls_score' : cls_scores[i].item(),
                    'score'     : scores[i].item() * cls_scores[i].item(),
                    'cx': cx, 'cy': cy, 'w': w, 'h': h,
                    'cx_px': cx * img_w, 'cy_px': cy * img_h,
                    'w_px' : w * img_w,  'h_px' : h * img_h,
                    'mask'  : full_masks[i],
                })
            results.append(instances)
        return results

    def param_count(self):
        return sum(p.numel() for p in self.parameters() if p.requires_grad)

    def __repr__(self):
        return (
            f'Segmenta(\n'
            f'  backbone = {self.cfg["backbone"]}\n'
            f'  fpn_ch   = {self.cfg.get("fpn_ch", 256)}\n'
            f'  classes  = {self.cfg["num_classes"]}\n'
            f'  img      = {self.cfg["img_h"]}×{self.cfg["img_w"]}\n'
            f'  params   = {self.param_count()/1e6:.2f}M\n'
            f')'
        )
