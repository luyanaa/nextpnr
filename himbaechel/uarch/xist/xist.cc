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
    }

    void pack() override
    {
        // The converter already turns IBUF/OBUF into PIO cells; tell nextpnr
        // which cell pin is the package pad so it can trim its synthetic IOBs.
        const pool<CellTypePort> top_ports{
                CellTypePort(id_PIO, id_PAD),
        };
        h.remove_nextpnr_iobs(top_ports);
        pack_constants();
        pack_lut_dff();
    }

    // The vendor slice muxes take GND as one of their four inputs (BFD:
    // ``arcval(GND,M0)``), so an unconnected LUT input reads 0.  Constants
    // that drive a LUT input are therefore folded into the INIT and the port
    // is dropped; any other constant use has no source in the fabric and is
    // rejected instead of silently producing a wrong bitstream.
    void pack_constants()
    {
        std::vector<IdString> dead_cells;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != id_VCC && ci->type != id_GND)
                continue;
            NetInfo *net = ci->getPort(id_Y);
            if (net != nullptr) {
                net->constant_value = (ci->type == id_VCC) ? id_VCC : id_GND;
                ci->disconnectPort(id_Y);
            }
            dead_cells.push_back(ci->name);
        }
        for (IdString name : dead_cells)
            ctx->cells.erase(name);


        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != id_LUT4)
                continue;
            for (int i = 0; i < 4; i++) {
                IdString port = ctx->idf("I[%d]", i);
                auto p = ci->ports.find(port);
                if (p == ci->ports.end() || p->second.net == nullptr)
                    continue;
                NetInfo *net = p->second.net;
                if (net->constant_value != id_VCC && net->constant_value != id_GND)
                    continue;
                if (ci->params.count(id_INIT) == 0)
                    log_error("LUT %s has no INIT\n", ctx->nameOf(ci));
                int64_t init = ci->params.at(id_INIT).intval;
                int64_t folded = 0;
                for (int b = 0; b < 16; b++) {
                    int src = (net->constant_value == id_GND) ? (b & ~(1 << i))
                                                              : (b | (1 << i));
                    if ((init >> src) & 1)
                        folded |= int64_t(1) << b;
                }
                ci->params[id_INIT] = Property(folded, 16);
                ci->disconnectPort(port);
            }
        }

        // Anything still on a constant net cannot be driven by the fabric.
        for (auto &net : ctx->nets) {
            NetInfo *ni = net.second.get();
            if (ni->constant_value == IdString() || ni->users.entries() == 0)
                continue;
            for (auto &usr : ni->users)
                log_error("constant net %s drives %s/%s; XIST has no VCC source\n",
                          ctx->nameOf(ni), ctx->nameOf(usr.cell),
                          usr.port.c_str(ctx));
        }
    }

    // nextpnr clusters are built directly: the root cell names the cluster
    // (ClusterId is an IdString) and children record their bel offset relative
    // to the root.  Every PLC tile has the same z layout - LUT<n> at
    // 4s+{0,1} and its DFF<n> at 4s+{2,3} - so a DFF is attached to the LUT
    // driving its D net at a fixed +2 z offset.
    static constexpr int LUT_DFF_DZ = 2;

    // A DFF whose D comes straight from a pad or another register has no LUT
    // in its bank.  The slice only feeds DI from its own output mux, so insert
    // the identity LUT the vendor would use (INIT = 0xAAAA, buffer on I[0])
    // and then cluster it like any other LUT/DFF pair.
    CellInfo *make_dff_lut(CellInfo *dff, NetInfo *d_net)
    {
        CellInfo *lut = ctx->createCell(ctx->idf("%s_DLUT", dff->name.c_str(ctx)), id_LUT4);
        IdString i0 = ctx->idf("I[0]");
        lut->addInput(i0);
        lut->addOutput(id_F);
        lut->params[id_INIT] = Property(int64_t(0xAAAA), 16);
        NetInfo *f = ctx->createNet(ctx->idf("%s_DF", dff->name.c_str(ctx)));
        lut->connectPort(i0, d_net);
        lut->connectPort(id_F, f);
        dff->disconnectPort(id_D);
        dff->connectPort(id_D, f);
        return lut;
    }

    int pack_lut_dff()
    {
        int paired = 0, unpaired = 0;
        std::vector<CellInfo *> dffs;
        for (auto &cell : ctx->cells)
            if (cell.second->type == id_DFF && cell.second->cluster == ClusterId())
                dffs.push_back(cell.second.get());

        for (CellInfo *dff : dffs) {
            // The DFF data input must come from a LUT in the same bank: the
            // slice DI wire is driven only by that bank's LUT (F -> DI pip).
            NetInfo *d_net = dff->getPort(id_D);
            if (d_net == nullptr) {
                ++unpaired;
                continue;
            }
            CellInfo *lut = nullptr;
            if (d_net->driver.cell != nullptr && d_net->driver.cell->type == id_LUT4)
                lut = d_net->driver.cell;
            if (lut == nullptr) {
                lut = make_dff_lut(dff, d_net);
                d_net = dff->getPort(id_D);
            }
            if (lut->cluster != ClusterId()) {
                // one LUT can only feed its own bank DFF; a shared LUT would
                // need replication, which is not implemented
                ++unpaired;
                continue;
            }

            lut->cluster = lut->name;
            lut->constr_abs_z = false;
            dff->cluster = lut->name;
            dff->constr_x = 0;
            dff->constr_y = 0;
            dff->constr_z = LUT_DFF_DZ;
            dff->constr_abs_z = false;
            lut->constr_children.push_back(dff);
            ++paired;
        }
        if (paired || unpaired)
            log_info("XIST: packed %d LUT+DFF slices, %d DFF with no LUT source\n",
                     paired, unpaired);
        return paired;
    }

    bool isBelLocationValid(BelId bel, bool explain_invalid) const override
    {
        IdString bel_type = ctx->getBelType(bel);
        if (bel_type != id_DFF)
            return true;

        // A clustered DFF is always placed together with the LUT that feeds
        // its D input, so the data-path rule below cannot fail for it.
        const CellInfo *bound = ctx->getBoundBelCell(bel);
        if (bound == nullptr)
            return true;   // empty bel: nothing to validate yet
        // CLK/CE/LSR muxes are slice-wide: both DFF banks must use the same
        // clock, and either both use the same enable/reset net or neither
        // does.  (A single used bank is fine - nothing to compare against.)
        {
            auto bel_name = ctx->getBelName(bel);
            std::string name = bel_name[1].str(ctx);
            if (!name.empty() && isdigit(name.back())) {
                int bank = name.back() - '0';
                auto sibling = ctx->getBelByName(IdStringList::concat(
                        bel_name[0], ctx->idf("DFF%d", bank ^ 1)));
                const CellInfo *other = ctx->getBoundBelCell(sibling);
                if (other != nullptr && other->type == id_DFF) {
                    for (IdString port : {id_CLK, id_CE, id_R, id_S}) {
                        if (bound->getPort(port) != other->getPort(port)) {
                            if (explain_invalid)
                                log_error("DFF %s and %s are in one slice: their "
                                          "shared %s control nets differ\n",
                                          ctx->nameOf(bound), ctx->nameOf(other),
                                          port.c_str(ctx));
                            return false;
                        }
                    }
                }
            }
        }

        {
            // The DFF data input is the slice output mux (OFX -> DI), which is
            // fed by the same bank's LUT (or the carry chain).  Routing cannot
            // reach DI directly, so a DFF with no used LUT in its bank is not
            // implementable.
            auto bel_name = ctx->getBelName(bel);
            const std::string name = bel_name[1].str(ctx);
            if (!name.empty() && isdigit(name.back())) {
                const int bank = name.back() - '0';
                const auto lut = ctx->getBelByName(
                        IdStringList::concat(bel_name[0], ctx->idf("LUT%d", bank)));
                if (lut != BelId() && ctx->checkBelAvail(lut)) {
                    if (explain_invalid)
                        log_error("DFF %s needs LUT%d in the same slice bank: its "
                                  "D input is the slice output mux (OFX->DI), which "
                                  "routing cannot drive.\n",
                                  ctx->getBelName(bel).str(ctx).c_str(), bank);
                    return false;
                }
            }
        }
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
