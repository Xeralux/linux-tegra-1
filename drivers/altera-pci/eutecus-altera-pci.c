/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Project:     PCIe pilot with Raggedstone 4 (refs #3992)
 * Purpose:     PCIe communication between the Raggedstone 4 and a PCI root port
 * Author:      György Kövesdi <gyorgy.kovesdi@eutecus.com>
 * License:     Eutecus Proprietary
 * Comments:    
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "eutecus-altera-pci.h"

#include <linux/string.h>
#include <linux/moduleparam.h>

#ifdef MODULE
static char version_string[] = MY_MODULE_NAME ": PCI Communication driver v" DRV_VERSION;
#endif

dbg_info dbg;
int debug;

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, " enable debug messages (1=generic, 2=file, 4=resource, 8=config, 16=calltrace, 32=create device files, 64=memory related, 128=video related)");

/* This function increments/decrements the usage counter of the PCI bus module, preventing
 * it from removing while this module is active.
 * Finds the fist module in the bus hierarchy to register. */
static int register_owner(struct device * dev, struct module * mod)
{
    int result;

    ENTER();

    if (!dev) {
        LEAVE();
        return 1; // Could not find module
    }

    DEBUG(files, "%sregistering '%s'...\n", mod ? "" : "un", dev->kobj.name);

    if (dev->class) {
        DEBUG(files, " - Class name: '%s'\n", dev->class->name);
    }

    if (dev->driver) {
        struct device_driver * drv = dev->driver;
        DEBUG(files, " - Driver name: '%s'\n", drv->name);
        if (drv->owner) {
            DEBUG(files, " -- Module name: '%s'\n", drv->owner->name);
            if (mod) {
                if (try_module_get(drv->owner)) {
                    // Note: ref_module() does not worth to call here, because the module name is not
                    //       displayed by lsmod (probably due to missing dependency), and the refcount
                    //       would be increased by 2 this way.
                    result = 0; // ref_module(mod, drv->owner);
                    DEBUG(files, " -- registered.\n");
                } else {
                    result = -1; // Failed
                    DEBUG(files, " -- cannot register!\n");
                }
            } else {
                module_put(drv->owner);
                result = 0; // Succeeded
                DEBUG(files, " -- unregistered.\n");
            }
            LEAVE_V("%d", result);
            return result;  // No more iteration
        } else {
            DEBUG(files, " -- no owner\n");
        }
    } else {
        DEBUG(files, " - no driver\n");
    }

    result = register_owner(dev->parent, mod);

    LEAVE_V("%d", result);

    return result;
}

static int altera_pci_probe(struct pci_dev * dev, const struct pci_device_id * ent)
{
    u32 d;
    int rc;

    ENTER();

    printk("Probing device " MY_MODULE_NAME "...\n");

    if (dev->driver) { // Sanity check
        rc = register_owner(dev->dev.parent, dev->driver->driver.owner);

        if (rc > 0) {
            MSG("INFO: No parent module found.\n");
        } else if (rc < 0) {
            ERROR("parent module registration error!\n");
        }
    }

    pci_set_master(dev);

    do {
        rc = pci_enable_device(dev);
        if (rc) {
            ERROR("could not enable PCI device!\n");
            break;
        }

        do {
            struct eutecus_pci_data * data;

            rc = pci_request_regions(dev, MY_MODULE_NAME);
            if (rc) {
                ERROR("could not request PCI regions!\n");
                break;
            }

            if (dbg.resources) {
                int i;
                for (i = 0; i < DEVICE_COUNT_RESOURCE; ++i) {
                    printk(MY_MODULE_NAME ": Altera PCI (%d/%d) resource #%2d: %p to %p, len=%p\n", ent->vendor, ent->device, i, (void*)pci_resource_start(dev, i), (void*)pci_resource_end(dev, i), (void*)pci_resource_len(dev, i));
                }
            }

            if (dbg.config) {
                int i;
                for (i = 0; i < 128; ++i) {
                    rc = pci_read_config_dword(dev, i*4, &d);
                    if (rc) {
                        ERROR("PCI config problem at entry #%d (err=%d)\n", i, rc);
                        break;
                    }
                    printk(MY_MODULE_NAME ": PCI config %3d: %#010x\n", i, d);
                }
                if (rc) {
                    break;
                }
            }

            data = kzalloc(sizeof(struct eutecus_pci_data), GFP_KERNEL);
            if (!data) {
                ERROR("could not alloc kernel data!\n");
                rc = -ENOMEM;
                break;
            }

            rc = init_cfile(data, dev);
            if (rc) {
                ERROR("device initialization failed\n");
                kfree(data);
                break;
            }

            pci_set_drvdata(dev, data);

            do {
                rc = altera_v4l2_initialize(dev);
                if (rc) {
                    ERROR("could not initialize V4l2 part\n");
                    break;
                }

                do {
                    char tmp[200];

                    data->irq = dev->irq;
                    rc = request_irq(dev->irq, eutecus_pci_isr, IRQF_SHARED, MY_MODULE_NAME, dev);
                    if (rc) {
                        ERROR("could not request interrupt %d\n", (int)dev->irq);
                        break;
                    }

                    if (!dbg.level) {
                        strcpy(tmp, "no debug");
                    } else {
                        strcpy(tmp, "with debug:");
                        if (dbg.generic) {
                            strcat(tmp, " generic");
                        }
                        if (dbg.files) {
                            strcat(tmp, " files");
                        }
                        if (dbg.resources) {
                            strcat(tmp, " resources");
                        }
                        if (dbg.config) {
                            strcat(tmp, " config");
                        }
                        if (dbg.calltrace) {
                            strcat(tmp, " calltrace");
                        }
                        if (dbg.memory) {
                            strcat(tmp, " memory");
                        }
                        if (dbg.video) {
                            strcat(tmp, " video");
                        }
                    }

                    printk(MY_MODULE_NAME ": driver loaded successfully (%s) PCI dev=%p, int #%d\n", tmp, dev, (int)dev->irq);

                    if (!dbg.level) {
                        NODEBUG("WARNING: debug is not compiled in, no messages will be displayed.\n");
                    }

                    // Clear the interrupt flag for safety:
                    interrupt_acknowledge_2_RS4(data);

                    // Send an interrupt to fix the initial status of the other side. The stream is not active, at least this information must be sent now.
                    interrupt_request_2_RS4(data);

                    return 0;   // Normal return point

                    // ---------------- Error handling below: -------------------------------------------------------------

                } while (0);

                altera_v4l2_destroy(dev);

            } while (0);

            pci_set_drvdata(dev, NULL);
            destroy_cfile(data);

        } while (0);

        pci_release_regions(dev);
        pci_disable_device(dev);

    } while (0);

    if (dev->driver) {
        register_owner(dev->dev.parent, 0);
    }

    ERROR("driver probe returned an error %d\n", rc);

    LEAVE_V("%d", rc);
    return rc;
}

static void altera_pci_remove(struct pci_dev * dev)
{
    struct eutecus_pci_data * data = pci_get_drvdata(dev);

    ENTER();

    if (data) {
        if (data->irq) {
            free_irq(data->irq, dev);
            data->irq = 0;
        }

        altera_v4l2_destroy(dev);

        pci_set_drvdata(dev, NULL);
        destroy_cfile(data);
    }

    pci_release_regions(dev);
    pci_disable_device(dev);

    if (dev->driver) {
        register_owner(dev->dev.parent, 0);
    }

    printk(MY_MODULE_NAME ": driver removed.\n");

    LEAVE();
}

static DEFINE_PCI_DEVICE_TABLE(altera_pci_tbl) = {
    { PCI_DEVICE(PCI_VENDOR_ID_ALTERA, PCI_DEVICE_ID_EPDE), },
    { },
};
MODULE_DEVICE_TABLE(pci, altera_pci_tbl);

static struct pci_driver altera_pci_driver = {
    .name         = MY_MODULE_NAME,
    .id_table     = altera_pci_tbl,
    .probe        =	altera_pci_probe,
    .remove       = altera_pci_remove,
};

static int __init altera_pci_module_init(void)
{
    int result;
    dbg.level = debug;
    pr_info("registering " MY_MODULE_NAME "...\n");
    result = pci_register_driver(&altera_pci_driver);
    pr_info("%s registered (%d)\n", version_string, result);
    return result;
}

static void __exit altera_pci_module_exit(void)
{
    pci_unregister_driver(&altera_pci_driver);
    pr_info("%s unregistered\n", version_string);
}

module_init(altera_pci_module_init);
module_exit(altera_pci_module_exit);

MODULE_DESCRIPTION("Altera PCI device communication module");
MODULE_AUTHOR("György Kövesdi <gyorgy.kovesdi@eutecus.com>");
MODULE_LICENSE("GPL");

/* * * * * * * * * * * * * End - of - File * * * * * * * * * * * * * * */
