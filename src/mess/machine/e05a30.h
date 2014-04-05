/***************************************************************************

    E05A30 Gate Array (used in the Epson ActionPrinter 2000)

    license: BSD-3-Clause
    copyright-holders: Ramiro Polla

***************************************************************************/

#ifndef __E05A30_H__
#define __E05A30_H__


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

struct e05a30_interface
{
	devcb_write8 m_pf_stepper_cb;
	devcb_write8 m_cr_stepper_cb;
};


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

class e05a30_device : public device_t, public e05a30_interface
{
public:
	e05a30_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);
	~e05a30_device() {}

	DECLARE_WRITE8_MEMBER( write );
	DECLARE_READ8_MEMBER( read );

protected:
	// device-level overrides
	virtual void device_config_complete();
	virtual void device_start();
	virtual void device_reset();

private:

	void update_printhead(int pos, UINT8 data);
	void update_pf_stepper(UINT8 data);
	void update_cr_stepper(UINT8 data);

	/* port 0x05 and 0x06 (9-bit) */
	UINT16 m_printhead;
	/* port 0x07 (4-bit) */
	UINT8 m_pf_stepper;
	/* port 0x08 (4-bit) */
	UINT8 m_cr_stepper;

	/* callbacks */
	devcb_resolved_write8 m_pf_stepper_func;
	devcb_resolved_write8 m_cr_stepper_func;
};

extern const device_type E05A30;


#define MCFG_E05A30_ADD(_tag, _intf) \
	MCFG_DEVICE_ADD(_tag, E05A30, 0) \
	MCFG_DEVICE_CONFIG(_intf)


#endif /* __E05A30_H__ */
