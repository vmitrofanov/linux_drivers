#include "bb_drv.h"
#include "bb_eth.h"

int poll_rx(struct napi_struct *napi, int weight)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);


	return 0;
}

int poll_tx(struct napi_struct *napi, int weight)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}
