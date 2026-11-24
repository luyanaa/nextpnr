/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  XIST (XiST HqFpga) microarchitecture bring-up.
 *
 *  This is a minimal uarch: it loads the chipdb produced by
 *  apicula/xist/chipdb.py and implements bel buckets/validity for the
 *  LUT4/DFF/PIO cells.  Packing and the bitstream writer are still TODO;
 *  the routing graph currently contains only pips observed in vendor XPN
 *  dumps.
 */

#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "util.h"
#include <cstdlib>

#include "himbaechel_helpers.h"

#define GEN_INIT_CONSTIDS
#define HIMBAECHEL_CONSTIDS "uarch/xist/constids.inc"
#define HIMBAECHEL_GFXIDS "uarch/xist/gfxids.inc"
#define HIMBAECHEL_UARCH xist
#include "himbaechel_constids.h"
#include "himbaechel_gfxids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
struct XistImpl : HimbaechelAPI
{
    ~XistImpl() {};

    po::options_description getUArchOptions()
    {
        po::options_description specific("XIST specific options");
        return specific;
    }

    void init_database(Arch *arch) override
    {
        init_uarch_constids(arch);
        arch->load_chipdb("xist/chipdb-SL2-12E-8F256C.bin");
        arch->set_speed_grade("8");
    }

    void init(Context *ctx) override
    {
        h.init(ctx);
        HimbaechelAPI::init(ctx);
        if (std::getenv("XIST_DUMP")) {
            const std::pair<const char *, const char *> wires[] = {
                {"X19Y39", "JDIA"}, {"X19Y20", "CLK"}, {"X19Y20", "CIB0"}};
            for (auto &tw : wires) {
                WireId w = ctx->getWireByName(
                    IdStringList::concat(ctx->id(tw.first), ctx->id(tw.second)));
                log_info("DUMP wire %s/%s id=%d\n", tw.first, tw.second, w.index);
                for (auto pip : ctx->getPipsDownhill(w))
                    log_info("  dh %s/%s -> %s\n", tw.first, tw.second,
                             ctx->getWireName(ctx->getPipDstWire(pip)).str(ctx).c_str());
                for (auto pip : ctx->getPipsUphill(w))
                    log_info("  uh %s/%s -> %s\n", tw.first, tw.second,
                             ctx->getWireName(ctx->getPipSrcWire(pip)).str(ctx).c_str());
            }
        }
    }

    void pack() override
    {
        // The converter already turns IBUF/OBUF into PIO cells; tell nextpnr
        // which cell pin is the package pad so it can trim its synthetic IOBs.
        const pool<CellTypePort> top_ports{
                CellTypePort(id_PIO, id_PAD),
        };
        h.remove_nextpnr_iobs(top_ports);
        // Constants and LUT/DFF pairing are TODO.
    }

    bool isBelLocationValid(BelId bel, bool explain_invalid) const override
    {
        (void)bel;
        (void)explain_invalid;
        return true;
    }

    IdString getBelBucketForCellType(IdString cell_type) const override
    {
        if (cell_type.in(id_IBUF, id_OBUF))
            return id_PIO;
        return cell_type;
    }

    bool isValidBelForCellType(IdString cell_type, BelId bel) const override
    {
        IdString bel_type = ctx->getBelType(bel);
        if (bel_type == id_PIO)
            return cell_type.in(id_PIO, id_IBUF, id_OBUF);
        return bel_type == cell_type;
    }

  private:
    HimbaechelHelpers h;
};
} // namespace

struct XistArch : HimbaechelArch
{
    XistArch() : HimbaechelArch("xist") {};
    bool match_device(const std::string &device) override
    {
        return device.rfind("SL2", 0) == 0;
    }
    std::unique_ptr<HimbaechelAPI> create(const std::string &device) override
    {
        (void)device;
        return std::make_unique<XistImpl>();
    }
} xistArch;

NEXTPNR_NAMESPACE_END
