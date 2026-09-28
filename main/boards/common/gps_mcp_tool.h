#ifndef _GPS_MCP_TOOL_H_
#define _GPS_MCP_TOOL_H_

#include "mcp_server.h"
#include "ml307_gps.h"
#include <memory>

/// Reusable MCP tool for GPS voice control (enable/disable/query)
class GpsMcpTool {
public:
    GpsMcpTool(std::shared_ptr<Ml307Gps> gps);

    /// Register the GPS control tool with the MCP server
    void Initialize();

private:
    std::shared_ptr<Ml307Gps> gps_;

    ReturnValue HandleGpsControl(const PropertyList& properties);
};

#endif  // _GPS_MCP_TOOL_H_
